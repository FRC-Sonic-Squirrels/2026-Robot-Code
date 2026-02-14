package frc.robot.subsystems.climber;

import com.ctre.phoenix6.signals.NeutralModeValue;
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
  private final String ROOT_TABLE = "Climber";

  public enum ClimberGoal {
    ClimbAuto, AutoDismount, L1Climb, L2Climb, L3Climb, Idle, Override
  }

  private enum ClimberState {
    Idle, Override, L1Extend, L1Retract, AutoDismount, AutoRetract, L2Extend, L2Retract, L3Extend, L3Retract, Lock
  }

  public static class ElevatorLogs extends AutoRegisterLoggers {
    public LoggerEntry.Decimal appliedVolts;
    public LoggerEntry.Decimal currentAmps;
    public LoggerEntry.Decimal tempCelsius;
    public LoggerEntry.Decimal velocityDegreesPerSecond;
    public LoggerEntry.Decimal targetHeightInches;
  }

  public static class Logs extends AutoRegisterLoggers {
    public ElevatorLogs innerClimber;
    public ElevatorLogs outerClimber;

    public LoggerEntry.Decimal servoTargetAngle;

  }

  public final Logs logs = LoggerGroup.root.newAutoLogger(Logs.class, ROOT_TABLE);

  // Execution timing
  private final ExecutionTiming climberTiming = new ExecutionTiming(ROOT_TABLE);

  public static class Tunables extends AutoRegisterTunables {
    public LoggedTunableNumber climberKP;
    public LoggedTunableNumber climberKD;
    public LoggedTunableNumber climberKG;

    public LoggedTunableNumber maxVelocityConfig;
    public LoggedTunableNumber climberTargetAccelerationConfig;

    @TunableEntry(defaultValue = 1.0)
    public LoggedTunableNumber toleranceHeight;

    public boolean hasChanged() {
      return climberKP.hasChanged() || climberKD.hasChanged() || climberKG.hasChanged();

    }
  }

  private final Tunables tunables = TunableNumberGroup.newAutoTunable(Tunables.class, ROOT_TABLE);

  private final ElevatorIO        innerElevatorIO;
  private final ElevatorIO        outerElevatorIO;
  private final ElevatorIO.Inputs innerElevatorInputs;
  private final ElevatorIO.Inputs outerElevatorInputs;

  private ClimberState climberState = ClimberState.Idle;
  private ClimberGoal  climberGoal  = ClimberGoal.Idle;
  private double       innerClimberHeight;
  private double       outerClimberHeight;

  // This also acts as its current angle because the servo cannot tell us where it
  // is
  private Rotation2d servoTargetAngle = Constants.ClimberConstants.SERVO_LOCK_ANGLE;

  /** Creates a new ClimberSubsystem. */
  public Climber(ElevatorIO innerIO, ElevatorIO outerIO) {
    this.innerElevatorIO     = innerIO;
    this.outerElevatorIO     = outerIO;
    this.innerElevatorInputs = new ElevatorIO.Inputs(logs.group);
    this.outerElevatorInputs = new ElevatorIO.Inputs(logs.group);

    if (Constants.RobotMode.getRobot() == RobotType.ROBOT_2025_RETIRED_HOLO) {
      tunables.climberKP.initDefault(4);
      tunables.climberKD.initDefault(0);
      tunables.climberKG.initDefault(0.0);

      tunables.maxVelocityConfig.initDefault(10);
      tunables.climberTargetAccelerationConfig.initDefault(10);

    } else if (Constants.RobotMode.getRobot() == RobotType.ROBOT_SIMBOT) {
      tunables.climberKP.initDefault(.02);
      tunables.climberKD.initDefault(0);
      tunables.climberKG.initDefault(0.0);

      tunables.maxVelocityConfig.initDefault(40);
      tunables.climberTargetAccelerationConfig.initDefault(80);
    } else {
      tunables.climberKP.initDefault(4);
      tunables.climberKD.initDefault(0);
      tunables.climberKG.initDefault(0.0);
    }

    setConstants();
    setServoAngle(ClimberConstants.SERVO_UNLOCK_ANGLE);
  }

  @Override
  public void periodic() {
    try (var ignored = climberTiming.start()) {
      // Climber logging
      innerElevatorIO.updateElevatorInputs(innerElevatorInputs);
      outerElevatorIO.updateElevatorInputs(innerElevatorInputs);

      logs.servoTargetAngle.info(servoTargetAngle);

      // Updating tunable numbers
      if (tunables.hasChanged()) {
        setConstants();
      }

      switch (climberState) {
        case Override:
          break;

        case Idle:
          setInnerTargetHeight(0);
          setOuterTargetHeight(0);
          break;

        case Lock:
          setServoAngle(ClimberConstants.SERVO_LOCK_ANGLE);
          break;

        case L1Extend:
          setOuterTargetHeight(ClimberConstants.L1_CLIMB_HEIGHT);
          if (isOuterClimberAtTargetHeight(ClimberConstants.L1_CLIMB_HEIGHT)) {
            setOuterTargetHeight(ClimberConstants.OUTER_RETRACTION_HEIGHT);
            climberState = ClimberState.L1Retract;
          }
          break;

        case L1Retract:
          if (isOuterClimberAtTargetHeight(ClimberConstants.OUTER_RETRACTION_HEIGHT)) {
            setOuterTargetHeight(0);
            switch (climberGoal) {

              case Override:
                break;

              case Idle:
                break;

              case ClimbAuto:
                break;

              case L1Climb:
                climberState = ClimberState.Lock;
                break;

              default:
                climberState = ClimberState.L2Extend;
                break;
            }
          }
          break;

        case AutoDismount:
          setOuterTargetHeight(ClimberConstants.L1_CLIMB_HEIGHT);
          break;

        case AutoRetract:
          setOuterTargetHeight(0);
          if (isOuterClimberAtTargetHeight(0)) {
            climberState = ClimberState.Idle;
          }
          break;

        case L2Extend:
          setInnerTargetHeight(ClimberConstants.L2_CLIMB_HEIGHT);
          if (isInnerClimberAtTargetHeight(ClimberConstants.L2_CLIMB_HEIGHT)) {
            setInnerTargetHeight(ClimberConstants.INNER_RETRACTION_HEIGHT);
            climberState = ClimberState.L2Retract;
          }
          break;

        case L2Retract:
          if (isInnerClimberAtTargetHeight(ClimberConstants.INNER_RETRACTION_HEIGHT)) {
            setInnerTargetHeight(0);
            switch (climberGoal) {

              case Override:
                break;

              case Idle:
                break;

              case ClimbAuto:
                break;

              case AutoDismount:
                break;

              case L1Climb:
                break;

              case L2Climb:
                climberState = ClimberState.Lock;
                break;

              default:
                climberState = ClimberState.L3Extend;
                break;
            }
          }
          break;

        case L3Extend:
          setOuterTargetHeight(ClimberConstants.L3_CLIMB_HEIGHT);
          if (isOuterClimberAtTargetHeight(ClimberConstants.L3_CLIMB_HEIGHT)) {
            setOuterTargetHeight(ClimberConstants.OUTER_RETRACTION_HEIGHT);
            climberState = ClimberState.L3Retract;
          }
          break;

        case L3Retract:
          if (isOuterClimberAtTargetHeight(ClimberConstants.OUTER_RETRACTION_HEIGHT)) {
            setOuterTargetHeight(0);
            switch (climberGoal) {

              case Override:
                break;

              case Idle:
                break;

              case ClimbAuto:
                break;

              case AutoDismount:
                break;

              case L1Climb:
                break;

              case L2Climb:
                break;

              case L3Climb:
                climberState = ClimberState.Lock;
                break;
            }
          }
        default:
          break;
      }
    }
  }

  // setters

  public void setClimberGoal(ClimberGoal goal) {
    climberGoal = goal;
    if (climberState != ClimberState.Idle) {
      return;
    }
    switch (goal) {

      case Override:
        climberState = ClimberState.Override;
        break;

      case Idle:
        climberState = ClimberState.Idle;
        break;

      case ClimbAuto:
        climberState = ClimberState.L1Extend;
        break;

      case AutoDismount:
        climberState = ClimberState.AutoDismount;
        break;

      case L1Climb:
        climberState = ClimberState.L1Extend;
        break;

      case L2Climb:
        climberState = ClimberState.L1Extend;
        break;

      case L3Climb:
        climberState = ClimberState.L1Extend;
        break;

      default:
        break;
    }
  }

  private void setConstants() {
    double kP = tunables.climberKP.get();
    double kD = tunables.climberKD.get();
    double kG = tunables.climberKG.get();
    innerElevatorIO.setInnerClimberClosedLoopConstants(kP, kD, kG);
    innerElevatorIO.setOuterClimberClosedLoopConstants(kP, kD, kG);
  }

  public void setInnerTargetHeight(double height) {
    innerElevatorIO.setInnerClimberClosedLoopPosition(height);
  }

  public void setOuterTargetHeight(double height) {
    innerElevatorIO.setOuterClimberClosedLoopPosition(height);
  }

  public boolean setClimberNeutralMode(NeutralModeValue value) {
    return innerElevatorIO.setClimberNeutralMode(value);
  }

  public void setServoAngle(Rotation2d angle) {
    innerElevatorIO.setClimberServoAngle(angle);
    servoTargetAngle = angle;
  }

  public Rotation2d getServoAngle() {
    return servoTargetAngle;
  }

  // getters

  public boolean isInnerClimberAtTargetHeight(double target) {
    var error = innerElevatorInputs.innerClimber.position - target;
    return error <= tunables.toleranceHeight.get();
  }

  public boolean isOuterClimberAtTargetHeight(double target) {
    var error = innerElevatorInputs.outerClimber.position - target;
    return error <= tunables.toleranceHeight.get();
  }

  public double getInnerClimberHeight() {
    return innerClimberHeight;
  }

  public double getOuterClimberHeight() {
    return outerClimberHeight;
  }

  public Voltage getInnerClimberVoltage() {
    return Units.Volts.of(innerElevatorInputs.innerClimber.appliedVolts);
  }

  public Voltage getOuterClimberVoltage() {
    return Units.Volts.of(innerElevatorInputs.outerClimber.appliedVolts);
  }

  public AngularVelocity getInnerClimberVelocity() {
    return Units.DegreesPerSecond.of(innerElevatorInputs.innerClimber.velocityDegreesPerSecond);
  }

  public AngularVelocity getOuterClimberVelocity() {
    return Units.DegreesPerSecond.of(innerElevatorInputs.outerClimber.velocityDegreesPerSecond);
  }
}
