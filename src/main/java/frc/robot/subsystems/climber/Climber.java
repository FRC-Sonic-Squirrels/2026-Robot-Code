package frc.robot.subsystems.climber;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Servo;
import frc.lib.team2930.*;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private final String ROOT_TABLE = "Climber";

  public enum ClimberGoal {
    ClimbAuto, AutoDismount, L1Climb, L2Climb, L3Climb, Idle, Override
  }

  private enum ClimberState {
    Idle, Override, L1Extend, L1Retract, AutoDismount, AutoRetract, L2Extend, L2Retract, L3Extend, L3Retract, Lock
  }

  public static class Logs extends AutoRegisterLoggers {
    public LoggerEntry.Decimal servoTargetAngle;
  }

  private Servo climberServo = new Servo(ClimberConstants.SERVO_CHANNEL);

  public final Logs logs = LoggerGroup.root.newAutoLogger(Logs.class, ROOT_TABLE);

  // Execution timing
  private final ExecutionTiming climberTiming = new ExecutionTiming(ROOT_TABLE);

  private final Elevator innerElevator;
  private final Elevator outerElevator;

  private ClimberState climberState = ClimberState.Idle;
  private ClimberGoal  climberGoal  = ClimberGoal.Idle;

  // This also acts as its current angle because the servo cannot tell us where it
  // is
  private Rotation2d servoTargetAngle = Constants.ClimberConstants.SERVO_LOCK_ANGLE;

  /** Creates a new ClimberSubsystem. */
  public Climber(ElevatorIO innerIO, ElevatorIO outerIO) {
    this.innerElevator = new Elevator(logs.group.subgroup("Inner"), innerIO);
    this.outerElevator = new Elevator(logs.group.subgroup("Outer"), outerIO);

    setServoAngle(ClimberConstants.SERVO_UNLOCK_ANGLE);
  }

  @Override
  public void periodic() {
    innerElevator.periodic();
    outerElevator.periodic();
    try (var ignored = climberTiming.start()) {
      // Climber logging

      logs.servoTargetAngle.info(servoTargetAngle);

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
    logs.servoTargetAngle.info(servoTargetAngle);
  }

  public void setInnerTargetHeight(double height) {
    innerElevator.setTargetHeight(height);
  }

  public void setOuterTargetHeight(double height) {
    outerElevator.setTargetHeight(height);
  }

  public boolean setClimberNeutralMode(NeutralModeValue value) {
    return innerElevator.setElevatorNeutralMode(value) && outerElevator.setElevatorNeutralMode(value);
  }

  public void setServoAngle(Rotation2d angle) {
    climberServo.setAngle(angle.getDegrees());
    servoTargetAngle = angle;
    logs.servoTargetAngle.info(servoTargetAngle);
  }

  public Rotation2d getServoAngle() {
    return servoTargetAngle;
  }

  // getters

  public boolean isInnerClimberAtTargetHeight(double target) {
    return innerElevator.isElevatorAtTargetHeight(target);
  }

  public boolean isOuterClimberAtTargetHeight(double target) {
    return outerElevator.isElevatorAtTargetHeight(target);
  }
}
