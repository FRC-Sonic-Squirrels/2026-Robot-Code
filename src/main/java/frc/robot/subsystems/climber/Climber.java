package frc.robot.subsystems.climber;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.*;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.RobotMode.RobotType;

public class Climber extends SubsystemBase {
  private static final String ROOT_TABLE = "ClimberWinch";

  public static class Logs extends AutoRegisterLoggers {
    public LoggerEntry.Decimal                winchAngle;
    public LoggerEntry.Decimal                winchAppliedVolts;
    public LoggerEntry.Decimal                winchCurrentAmps;
    public LoggerEntry.Decimal                winchTempCelsius;
    public LoggerEntry.Decimal                winchVelocityDegreesPerSecond;
    public LoggerEntry.EnumValue<ControlMode> winchControlMode;
    public LoggerEntry.Decimal                winchTargetAngleDegrees;
    public LoggerEntry.Decimal                servoTargetAngleDegrees;

  }

  public static final Logs logs = LoggerGroup.root.newAutoLogger(Logs.class, ROOT_TABLE);

  // Execution timing
  private static final ExecutionTiming winchTiming = new ExecutionTiming(ROOT_TABLE);

  // Tunable Numbers
  private static final TunableNumberGroup winchGroup = new TunableNumberGroup(ROOT_TABLE);

  private static final LoggedTunableNumber winchkP = winchGroup.build("kP");
  private static final LoggedTunableNumber winchkD = winchGroup.build("kD");
  private static final LoggedTunableNumber winchkG = winchGroup.build("kG");

  private static final LoggedTunableNumber maxVelocityConfig             = winchGroup.build("MaxVelocityConfig");
  private static final LoggedTunableNumber winchTargetAccelerationConfig = winchGroup.build("TargetAccelerationConfig");
  private static final LoggedTunableNumber toleranceDegrees              = winchGroup.build("ToleranceDegrees", 40);

  // Tunable numbers

  static {
    if (Constants.RobotMode.getRobot() == RobotType.ROBOT_2025_RETIRED_HOLO) {
      winchkP.initDefault(4);
      winchkD.initDefault(0);
      winchkG.initDefault(0.0);

      maxVelocityConfig.initDefault(10);
      winchTargetAccelerationConfig.initDefault(10);

    } else if (Constants.RobotMode.getRobot() == RobotType.ROBOT_SIMBOT) {

      winchkP.initDefault(.02);
      winchkD.initDefault(0);
      winchkG.initDefault(0.0);

      maxVelocityConfig.initDefault(40);
      winchTargetAccelerationConfig.initDefault(80);
    } else {
      winchkP.initDefault(4);
      winchkD.initDefault(0);
      winchkG.initDefault(0.0);
    }
  }

  private final ClimberIO        io;
  private final ClimberIO.Inputs winchInputs;

  private ControlMode winchControlMode        = ControlMode.OPEN_LOOP;
  private Rotation2d  winchTargetAngleDegrees = Constants.zeroRotation2d;

  // This also acts as its current angle because the servo cannot tell us where it
  // is
  private Rotation2d servoTargetAngle = Constants.ClimberConstants.SERVO_LOCK_ANGLE;

  /** Creates a new ClimberSubsystem. */
  public Climber(ClimberIO io) {
    this.io          = io;
    this.winchInputs = new ClimberIO.Inputs(logs.group);

    io.setWinchVoltage(0.0);

    setConstants();
    setServoAngle(ClimberConstants.SERVO_UNLOCK_ANGLE);
  }

  @Override
  public void periodic() {
    try (var ignored = winchTiming.start()) {
      // Winch logging
      io.updateWinchInputs(winchInputs);

      logs.winchAngle.info(winchInputs.winchPosition);
      logs.winchAppliedVolts.info(winchInputs.winchAppliedVolts);
      logs.winchCurrentAmps.info(winchInputs.winchCurrentAmps);
      logs.winchTempCelsius.info(winchInputs.winchTempCelsius);
      logs.winchVelocityDegreesPerSecond.info(winchInputs.winchVelocityDegreesPerSecond);
      logs.winchControlMode.info(winchControlMode);

      // Updating tunable numbers
      var hc = hashCode();
      if (winchkP.hasChanged(hc) || winchkD.hasChanged(hc) || winchkG.hasChanged(hc)) {
        setConstants();
      }
    }
    logs.servoTargetAngleDegrees.info(servoTargetAngle);
  }

  // setters

  private void setConstants() {
    io.setWinchClosedLoopConstants(winchkP.get(), winchkD.get(), winchkG.get());
  }

  public void setWinchAngle(Rotation2d angle) {
    angle = Rotation2d
        .fromDegrees(MathUtil.clamp(angle.getDegrees(), Constants.ClimberConstants.MIN_CLIMBER_ANGLE.getDegrees(),
            Constants.ClimberConstants.MAX_CLIMBER_ANGLE.getDegrees()));

    winchControlMode        = ControlMode.CLOSED_LOOP;
    winchTargetAngleDegrees = angle;
    io.setWinchClosedLoopPosition(angle);
    logs.winchTargetAngleDegrees.info(winchTargetAngleDegrees);
  }

  public void resetWinchSubsystem() {
    winchControlMode = ControlMode.OPEN_LOOP;
    io.setWinchVoltage(0);
  }

  public void setWinchVoltage(double percent) {
    winchControlMode = ControlMode.OPEN_LOOP;
    io.setWinchVoltage(percent);
  }

  public void resetWinchSensorToHomePosition() {
    io.resetWinchSensorPosition(Constants.ClimberConstants.MIN_CLIMBER_ANGLE);
  }

  public boolean setWinchNeutralMode(NeutralModeValue value) {
    return io.setWinchNeutralMode(value);
  }

  public void setServoAngle(Rotation2d angle) {
    io.setClimberServoAngle(angle);
    servoTargetAngle = angle;
  }

  public Rotation2d getServoAngle() {
    return servoTargetAngle;
  }

  // Winch getters

  public Rotation2d getWinchAngle() {
    return Rotation2d.fromDegrees(winchInputs.winchPosition);
  }

  public boolean isWinchAtTargetAngle() {
    return isWinchAtTargetAngle(winchTargetAngleDegrees);
  }

  public boolean isWinchAtTargetAngle(Rotation2d target, Rotation2d tolerance) {
    return GeometryUtil.isRotationAtTarget(Rotation2d.fromDegrees(winchInputs.winchPosition), target, tolerance);
  }

  public boolean isWinchAtTargetAngle(Rotation2d target) {
    return isWinchAtTargetAngle(target, Rotation2d.fromDegrees(toleranceDegrees.get()));
  }

  public Voltage getWinchVoltage() {
    return Units.Volts.of(winchInputs.winchAppliedVolts);
  }

  public AngularVelocity getWinchVelocity() {
    return Units.DegreesPerSecond.of(winchInputs.winchVelocityDegreesPerSecond);
  }
}
