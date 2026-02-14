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
  private static final String ROOT_TABLE = "Intake";

  public enum State {
    Idle, Stow, Down, Eject, Intake
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
  }

  public static class Logs extends AutoRegisterLoggers {
    public LoggerEntry.EnumValue<State> state;

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

    @TunableEntry(defaultValue = 20.0)
    public LoggedTunableNumber intakingFuelPivotAngle;
  }

  public final Logs logs = LoggerGroup.root.newAutoLogger(Logs.class, ROOT_TABLE);

  // Execution timing
  private final ExecutionTiming timing = new ExecutionTiming(ROOT_TABLE);

  // Tunable numbers
  private final Tunables tunables = TunableNumberGroup.newAutoTunable(Tunables.class, ROOT_TABLE);

  private final IntakeIO        io;
  private final IntakeIO.Inputs inputs = new IntakeIO.Inputs(logs.group);

  // Intake states
  private State state = State.Stow;

  private boolean intakeOverride = false;

  private double rollerTargetRPM;

  private ControlMode pivotControlMode = ControlMode.OPEN_LOOP;
  private Rotation2d  pivotTargetAngle = PivotConstants.HOME_POSITION;

  private ControlMode rollerControlMode = ControlMode.OPEN_LOOP;

  private Trigger stallDetected = new Trigger(() -> inputs.rollerVelocityRPM <= 50 && inputs.rollerAppliedVolts >= 0.5)
      .debounce(.1);

  /** Creates a new Intake. */
  public Intake(IntakeIO io) {
    this.io = io;

    // Motion constants
    if (Constants.RobotMode.getRobot() == RobotType.ROBOT_2025_RETIRED_HOLO) {
      tunables.roller.KS.initDefault(0);
      tunables.roller.KP.initDefault(0.4);
      tunables.roller.KV.initDefault(0.13);
      tunables.roller.targetAccelerationConfig.initDefault(200);

      tunables.pivot.KP.initDefault(60);
      tunables.pivot.KD.initDefault(0.3);
      tunables.pivot.KG.initDefault(0.4);

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
      logs.roller.controlMode.info(rollerControlMode);

      logs.pivot.angle.info(inputs.pivotPosition);
      logs.pivot.appliedVolts.info(inputs.pivotAppliedVolts);
      logs.pivot.currentAmps.info(inputs.pivotCurrentAmps);
      logs.pivot.tempCelsius.info(inputs.pivotTempCelsius);
      logs.pivot.velocityDegreesPerSecond.info(inputs.pivotVelocityDegreesPerSecond);
      logs.pivot.controlMode.info(pivotControlMode);

      logs.state.info(state);

      // Update tunable numbers

      if (tunables.roller.hasChanged()) {
        setRollerConstants();
      }
      if (tunables.pivot.hasChanged()) {
        setPivotConstants();
      }

      // Intake state machine
      if (!intakeOverride) {
        switch (state) {
          case Idle:
            setPivotVoltage(0);
            setRollerPercentOut(0);
            break;
          case Stow:
            setPivotAngle(PivotConstants.HOME_POSITION);
            setRollerPercentOut(0);
          case Down:
            setPivotAngle(computeDownAngle());
            setRollerPercentOut(0);
            break;
          case Eject:
            setRollerVelocity(tunables.ejectingFuelVelocity.get() * -1);
            break;
          case Intake:
            setRollerVelocity(tunables.intakingFuelVelocity.get());
            setPivotAngle(computeDownAngle());
            break;
          default:
            break;
        }
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

  public void setRollerPercentOut(double percent) {
    io.setRollerVoltage(percent * Constants.MAX_VOLTAGE);
    rollerControlMode = ControlMode.OPEN_LOOP;
  }

  public void setRollerVelocity(double revPerMin) {
    rollerControlMode = ControlMode.CLOSED_LOOP;
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

  public void setPivotAngle(Rotation2d angle) {
    Rotation2d targetAngle = Rotation2d.fromRadians(MathUtil.clamp(angle.getRadians(),
        PivotConstants.MIN_PIVOT_ANGLE.getRadians(), PivotConstants.MAX_PIVOT_ANGLE.getRadians()));

    pivotControlMode = ControlMode.CLOSED_LOOP;
    pivotTargetAngle = targetAngle;
    io.setPivotClosedLoopPosition(targetAngle);
    logs.pivot.targetAngleDegrees.info(targetAngle.getDegrees());
  }

  public void resetPivotSubsystem() {
    pivotControlMode = ControlMode.OPEN_LOOP;
    io.setPivotVoltage(0.0);
  }

  public void setPivotVoltage(double percent) {
    pivotControlMode = ControlMode.OPEN_LOOP;
    io.setPivotVoltage(percent);
  }

  public void resetPivotSensorToHomePosition() {
    io.resetPivotSensorPosition(PivotConstants.HOME_POSITION);
  }

  public boolean setPivotNeutralMode(NeutralModeValue value) {
    return io.setPivotNeutralMode(value);
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

  public Voltage getPivotVoltage() {
    return Units.Volts.of(inputs.pivotAppliedVolts);
  }

  public AngularVelocity getPivotVelocity() {
    return Units.DegreesPerSecond.of(inputs.pivotVelocityDegreesPerSecond);
  }

  public void setIntakeOverride(boolean intakeOverride) {
    this.intakeOverride = intakeOverride;
  }

  public boolean getIntakeOverride() {
    return intakeOverride;
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

  public void setState(State state) {
    this.state = state;
  }
}
