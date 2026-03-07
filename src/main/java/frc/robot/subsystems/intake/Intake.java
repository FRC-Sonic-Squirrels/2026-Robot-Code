// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.*;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants.PivotConstants;
import frc.robot.Constants.RobotMode.RobotType;

public class Intake extends SubsystemBase {
  public enum Goal {
    Stow, Down, Eject, Intake
  }

  public static class RollerLogs extends AutoRegisterLoggers {
    public LoggerEntry.Decimal                velocityRPM;
    public LoggerEntry.Decimal                currentAmps;
    public LoggerEntry.Decimal                tempCelsius;
    public LoggerEntry.Decimal                appliedVolts;
    public LoggerEntry.Decimal                targetVelocityRPM;
    public LoggerEntry.EnumValue<ControlMode> controlMode;
  }

  public static class PivotLogs extends AutoRegisterLoggers {
    public LoggerEntry.Decimal                angle;
    public LoggerEntry.Decimal                appliedVolts;
    public LoggerEntry.Decimal                currentAmps;
    public LoggerEntry.Decimal                tempCelsius;
    public LoggerEntry.Decimal                velocityDegreesPerSecond;
    public LoggerEntry.EnumValue<ControlMode> controlMode;
    public LoggerEntry.Decimal                targetAngleDegrees;
    public LoggerEntry.Decimal                targetAngle;
    public LoggerEntry.EnumValue<ControlMode> pivotControlMode;
  }

  public static class Logs extends AutoRegisterLoggers {
    public LoggerEntry.EnumValue<Goal> goal;

    public RollerLogs roller;
    public PivotLogs  pivot;
  }

  public static class PivotTunables extends AutoRegisterTunables {
    public LoggedTunableNumber KP;
    public LoggedTunableNumber KD;
    public LoggedTunableNumber KG;

    public LoggedTunableNumber maxVelocityConfig;
    public LoggedTunableNumber targetAccelerationConfig;

    @TunableEntry(defaultValue = 3.0)
    public LoggedTunableNumber toleranceDegrees;

    private boolean hasChanged() {
      return KP.hasChanged() || KD.hasChanged() || KG.hasChanged() || maxVelocityConfig.hasChanged()
          || targetAccelerationConfig.hasChanged();
    }
  }

  public static class RollerTunables extends AutoRegisterTunables {
    public LoggedTunableNumber KS;
    public LoggedTunableNumber KP;
    public LoggedTunableNumber KV;
    public LoggedTunableNumber targetAccelerationConfig;

    public boolean hasChanged() {
      return KS.hasChanged() || KP.hasChanged() || KV.hasChanged() || targetAccelerationConfig.hasChanged();
    }
  }

  public static class Tunables extends AutoRegisterTunables {
    public PivotTunables  pivot;
    public RollerTunables roller;

    @TunableEntry(defaultValue = 3000.0)
    public LoggedTunableNumber intakingFuelVelocity;

    @TunableEntry(defaultValue = 500.0)
    public LoggedTunableNumber ejectingFuelVelocity;

    @TunableEntry(defaultValue = 10)
    public LoggedTunableNumber intakingFuelPivotAngle;
  }

  public final Logs logs;

  private ControlMode pivotControlMode = ControlMode.OPEN_LOOP;

  // Execution timing
  private final ExecutionTiming timing;

  // Tunable numbers
  private final Tunables tunables;

  private final IntakeIO        io;
  private final IntakeIO.Inputs inputs;

  // Intake states
  private Goal goal = Goal.Stow;

  private double rollerTargetRPM;

  private Rotation2d pivotTargetAngle = PivotConstants.HOME_POSITION;

  private final Trigger stallDetected;

  /** Creates a new Intake. */
  public Intake(String name, IntakeIO io) {
    logs = LoggerGroup.root.newAutoLogger(Logs.class, name);

    // Execution timing
    timing = new ExecutionTiming(name);

    // Tunable numbers
    tunables = TunableNumberGroup.newAutoTunable(Tunables.class, name);

    this.io       = io;
    this.inputs   = new IntakeIO.Inputs(logs.group);
    stallDetected = new Trigger(() -> inputs.rollerVelocityRPM <= 50 && inputs.rollerAppliedVolts >= 0.5).debounce(.1);

    // Motion constants
    if (Constants.RobotMode.getRobot() == RobotType.ROBOT_2025_RETIRED_HOLO) {
      tunables.roller.KS.initDefault(0);
      tunables.roller.KP.initDefault(0.4);
      tunables.roller.KV.initDefault(0.13);
      tunables.roller.targetAccelerationConfig.initDefault(200);

      tunables.pivot.KG.initDefault(-2.2);
      tunables.pivot.KD.initDefault(5);
      tunables.pivot.KP.initDefault(50);

      tunables.pivot.maxVelocityConfig.initDefault(200);
      tunables.pivot.targetAccelerationConfig.initDefault(1500);
    } else if (Constants.RobotMode.isSimBot()) {
      tunables.roller.KS.initDefault(0);
      tunables.roller.KP.initDefault(0.00006);
      tunables.roller.KV.initDefault(0.0002);
      tunables.roller.targetAccelerationConfig.initDefault(0.0);

      tunables.pivot.KP.initDefault(120);
      tunables.pivot.KD.initDefault(0);
      tunables.pivot.KG.initDefault(0.0);

      tunables.pivot.maxVelocityConfig.initDefault(40);
      tunables.pivot.targetAccelerationConfig.initDefault(80);
    } else {
      tunables.roller.KS.initDefault(0);
      tunables.roller.KP.initDefault(0.4);
      tunables.roller.KV.initDefault(0.13);
      tunables.roller.targetAccelerationConfig.initDefault(200);

      tunables.pivot.KP.initDefault(60);
      tunables.pivot.KD.initDefault(0.3);
      tunables.pivot.KG.initDefault(0.4);

      tunables.pivot.maxVelocityConfig.initDefault(200);
      tunables.pivot.targetAccelerationConfig.initDefault(1500);
    }

    io.setPivotVoltage(0.0);

    setPivotConstants();

    io.setRollerVoltage(0.0);

    setRollerConstants();
  }

  @Override
  public void periodic() {
    try (var ignored = timing.start()) {
      // Logging
      io.updateInputs(inputs);
      logs.roller.velocityRPM.info(inputs.rollerVelocityRPM);
      logs.roller.currentAmps.info(inputs.rollerCurrentAmps);
      logs.roller.tempCelsius.info(inputs.rollerTempCelsius);
      logs.roller.appliedVolts.info(inputs.rollerAppliedVolts);
      logs.roller.targetVelocityRPM.info(rollerTargetRPM);

      logs.pivot.angle.info(inputs.pivotPosition);
      logs.pivot.appliedVolts.info(inputs.pivotAppliedVolts);
      logs.pivot.currentAmps.info(inputs.pivotCurrentAmps);
      logs.pivot.tempCelsius.info(inputs.pivotTempCelsius);
      logs.pivot.velocityDegreesPerSecond.info(inputs.pivotVelocityDegreesPerSecond);
      logs.pivot.targetAngle.info(pivotTargetAngle);
      logs.pivot.controlMode.info(pivotControlMode);

      logs.goal.info(goal);

      // Update tunable numbers

      if (tunables.roller.hasChanged()) {
        setRollerConstants();
      }
      if (tunables.pivot.hasChanged()) {
        setPivotConstants();
      }

      // Intake state machine
      switch (goal) {
        case Stow:
          setPivotUpCurrentLimit();
          setPivotAngle(PivotConstants.HOME_POSITION);
          setRollerVelocity(0);
          break;
        case Down:
          setPivotAngle(PivotConstants.MAX_PIVOT_ANGLE);
//          setPivotDownCurrentLimit();
//          setPivotVoltage(-2);
          setRollerVelocity(0);
          break;
        case Eject:
          setRollerVelocity(tunables.ejectingFuelVelocity.get() * -1);
          break;
        case Intake:
          setPivotAngle(PivotConstants.MAX_PIVOT_ANGLE);
//          setPivotDownCurrentLimit();
//          setPivotVoltage(-2);
          setRollerVelocity(tunables.intakingFuelVelocity.get());
          break;
        default:
          break;
      }
    }
  }

  private Rotation2d computeDownAngle() {
    return Rotation2d.fromDegrees(tunables.intakingFuelPivotAngle.get());
  }

  // Setters

  private void setRollerConstants() {
    var roller = tunables.roller;
    io.setRollerClosedLoopConstants(roller.KP.get(), roller.KV.get(), roller.KS.get(),
        roller.targetAccelerationConfig.get());
  }

  public void setRollerVelocity(double revPerMin) {
    io.setRollerVelocity(revPerMin);
    rollerTargetRPM = revPerMin;
    logs.roller.targetVelocityRPM.info(rollerTargetRPM);
  }

  private void setPivotConstants() {
    var pivot = tunables.pivot;

    var configs = new MotionMagicConfigs();
    configs.MotionMagicCruiseVelocity = pivot.maxVelocityConfig.get();
    configs.MotionMagicAcceleration   = pivot.targetAccelerationConfig.get();

    io.setPivotClosedLoopConstants(pivot.KP.get(), pivot.KD.get(), pivot.KG.get(), configs);
  }

  public void setPivotVoltage(double percent) {
    pivotControlMode = ControlMode.OPEN_LOOP;
    io.setPivotVoltage(percent);
  }

  public void setPivotDownCurrentLimit() {
    io.setPivotCurrentLimits(PivotConstants.LOW_SUPPLY_CURRENT_LIMIT, PivotConstants.LOW_STATOR_CURRENT_LIMIT);
  }

  public void setPivotUpCurrentLimit() {
    io.setPivotCurrentLimits(PivotConstants.HIGH_SUPPLY_CURRENT_LIMIT, PivotConstants.HIGH_STATOR_CURRENT_LIMIT);
  }

  public void setPivotAngle(Rotation2d angle) {
    Rotation2d targetAngle = Rotation2d.fromRadians(MathUtil.clamp(angle.getRadians(),
        PivotConstants.MIN_PIVOT_ANGLE.getRadians(), PivotConstants.MAX_PIVOT_ANGLE.getRadians()));
    pivotControlMode = ControlMode.CLOSED_LOOP;

    pivotTargetAngle = targetAngle;
    io.setPivotClosedLoopPosition(targetAngle);
    logs.pivot.targetAngleDegrees.info(targetAngle);
  }

  public void resetPivotSensorToHomePosition() {
    io.resetPivotSensorPosition(PivotConstants.HOME_POSITION);
  }

  public void setPivotNeutralMode(NeutralModeValue value) {
    io.setPivotNeutralMode(value);
  }

  // Getters

  public Current getRollerCurrentDraw() {
    return Units.Amps.of(inputs.rollerCurrentAmps);
  }

  public AngularVelocity getRollerVelocity() {
    return Units.RPM.of(inputs.rollerVelocityRPM);
  }

  public Rotation2d getPivotAngle() {
    return inputs.pivotPosition;
  }

  public boolean isStowed() {
    return GeometryUtil.isRotationAtTarget(inputs.pivotPosition, PivotConstants.HOME_POSITION,
        tunables.pivot.toleranceDegrees.get());
  }

  public AngularVelocity getPivotVelocity() {
    return Units.DegreesPerSecond.of(inputs.pivotVelocityDegreesPerSecond);
  }

  public Voltage getPivotVoltage() {
    return Units.Volts.of(inputs.pivotAppliedVolts);
  }

  public boolean rollerStallDetected() {
    return stallDetected.getAsBoolean();
  }

  public IntakeIOSim getSim() {
    if (io instanceof IntakeIOSim) {
      return (IntakeIOSim) io;
    }

    return null;
  }

  public void setGoal(Goal goal) {
    this.goal = goal;
  }
}
