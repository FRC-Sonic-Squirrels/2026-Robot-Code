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

  public static final Logs logs = LoggerGroup.root.newAutoLogger(Logs.class, ROOT_TABLE);

  // Execution timing
  private static final ExecutionTiming timing = new ExecutionTiming(ROOT_TABLE);

  // Logging
  private static final LoggerGroup logGroup = LoggerGroup.build(ROOT_TABLE);

  // Tunable numbers
  private static final TunableNumberGroup group = new TunableNumberGroup(ROOT_TABLE);

  private static final LoggedTunableNumber intakingVelocity = group.build("Intaking/Fuel/Velocity", 3000);

  private static final LoggedTunableNumber ejectingFuelVelocity   = group.build("Score/Fuel/Velocity", 500.0);
  private static final LoggedTunableNumber intakingFuelPivotAngle = group.build("Intaking/Fuel/AngleDeg", 20);

  // Roller
  private static final TunableNumberGroup  rollerSubgroup                 = group.subgroup("Roller");
  private static final LoggedTunableNumber rKS                            = rollerSubgroup.build("KS");
  private static final LoggedTunableNumber rKP                            = rollerSubgroup.build("KP");
  private static final LoggedTunableNumber rKV                            = rollerSubgroup.build("KV");
  private static final LoggedTunableNumber rollerTargetAccelerationConfig = rollerSubgroup
      .build("MaxAccelerationConstraint");

  // Pivot
  private static final TunableNumberGroup  pivotSubgroup = group.subgroup("Pivot");
  private static final LoggedTunableNumber pKP           = pivotSubgroup.build("KP");
  private static final LoggedTunableNumber pKD           = pivotSubgroup.build("KD");
  private static final LoggedTunableNumber pKG           = pivotSubgroup.build("KG");

  private static final LoggedTunableNumber pivotMaxVelocityConfig        = pivotSubgroup.build("MaxVelocityConfig");
  private static final LoggedTunableNumber pivotTargetAccelerationConfig = pivotSubgroup
      .build("TargetAccelerationConfig");
  private static final LoggedTunableNumber pivotToleranceDegrees         = pivotSubgroup.build("ToleranceDegrees", 3);

  // Motion constants
  static {
    if (Constants.RobotMode.getRobot() == RobotType.ROBOT_2025_RETIRED_HOLO) {
      rKS.initDefault(0);
      rKP.initDefault(0.4);
      rKV.initDefault(0.13);
      rollerTargetAccelerationConfig.initDefault(200);

      pKP.initDefault(60);
      pKD.initDefault(0.3);
      pKG.initDefault(0.4);

      pivotMaxVelocityConfig.initDefault(200);
      pivotTargetAccelerationConfig.initDefault(1500);
    } else if (Constants.RobotMode.isSimBot()) {
      rKS.initDefault(0);
      rKP.initDefault(0.00006);
      rKV.initDefault(0.0002);
      rollerTargetAccelerationConfig.initDefault(0.0);

      pKP.initDefault(120);
      pKD.initDefault(0);
      pKG.initDefault(0.0);

      pivotMaxVelocityConfig.initDefault(40);
      pivotTargetAccelerationConfig.initDefault(80);
    } else {
      rKS.initDefault(0);
      rKP.initDefault(0.4);
      rKV.initDefault(0.13);
      rollerTargetAccelerationConfig.initDefault(200);

      pKP.initDefault(60);
      pKD.initDefault(0.3);
      pKG.initDefault(0.4);

      pivotMaxVelocityConfig.initDefault(200);
      pivotTargetAccelerationConfig.initDefault(1500);
    }
  }

  private final IntakeIO        io;
  private final IntakeIO.Inputs inputs = new IntakeIO.Inputs(logGroup);

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

      var rhc = hashCode();
      if (rKS.hasChanged(rhc) || rKP.hasChanged(rhc) || rKV.hasChanged(rhc)
          || rollerTargetAccelerationConfig.hasChanged(rhc)) {
        setRollerConstants();
      }
      var phc = hashCode();
      if (pKP.hasChanged(phc) || pKD.hasChanged(phc) || pKG.hasChanged(phc) || pivotMaxVelocityConfig.hasChanged(phc)
          || pivotTargetAccelerationConfig.hasChanged(phc)) {
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
            setRollerVelocity(ejectingFuelVelocity.get() * -1);
            break;
          case Intake:
            setRollerVelocity(intakingVelocity.get());
            setPivotAngle(computeDownAngle());
            break;
          default:
            break;
        }
      }
    }
  }

  private Rotation2d computeDownAngle() {
    return Rotation2d.fromDegrees(intakingFuelPivotAngle.get());
  }

  // Setters

  private void setRollerConstants() {
    io.setRollerClosedLoopConstants(rKP.get(), rKV.get(), rKS.get(), rollerTargetAccelerationConfig.get());
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
    MotionMagicConfigs configs = new MotionMagicConfigs();
    configs.MotionMagicCruiseVelocity = pivotMaxVelocityConfig.get();
    configs.MotionMagicAcceleration   = pivotTargetAccelerationConfig.get();
    io.setPivotClosedLoopConstants(pKP.get(), pKD.get(), pKG.get(), configs);
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
        pivotToleranceDegrees.get());
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
