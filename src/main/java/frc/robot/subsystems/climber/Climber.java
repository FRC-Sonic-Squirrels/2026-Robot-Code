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

  public enum Goal {
    MountDismount, ClimbL1, ClimbL2, ClimbL3, Idle, Override, Lock
  }

  private enum ClimberState {
    Idle, Override, L1Retract, MountDismount, L2Extend, L2Retract, L3Extend, L3Retract, Lock
  }

  public static class Logs extends AutoRegisterLoggers {
    public LoggerEntry.EnumValue<ClimberState> state;
    public LoggerEntry.EnumValue<Goal>         goal;
    public LoggerEntry.Decimal                 servoTargetAngle;
  }

  private Servo climberServo = new Servo(ClimberConstants.SERVO_CHANNEL);

  public final Logs logs = LoggerGroup.root.newAutoLogger(Logs.class, ROOT_TABLE);

  // Execution timing
  private final ExecutionTiming climberTiming = new ExecutionTiming(ROOT_TABLE);

  private final Elevator innerElevator;
  private final Elevator outerElevator;

  private ClimberState climberState = ClimberState.Idle;
  private Goal         climberGoal  = Goal.Idle;

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

      logs.state.info(climberState);
      logs.goal.info(climberGoal);
      logs.servoTargetAngle.info(servoTargetAngle);

      switch (climberGoal) {

        case Override:
          climberState = ClimberState.Override;
          break;

        case Idle:
          climberState = ClimberState.Idle;
          break;

        case Lock:
          climberState = ClimberState.Lock;
          break;

        case MountDismount:
          climberState = ClimberState.MountDismount;
          break;

        case ClimbL1:
          climberState = ClimberState.L1Retract;
          break;

        case ClimbL2:
          climberState = ClimberState.L1Retract;
          break;

        case ClimbL3:
          climberState = ClimberState.L1Retract;
          break;

        default:
          break;
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

        case MountDismount:
          setOuterTargetHeight(ClimberConstants.L1_CLIMB_HEIGHT);
          break;

        case L1Retract:
          setOuterTargetHeight(ClimberConstants.OUTER_RETRACTION_HEIGHT);
          if (isOuterClimberAtTargetHeight(ClimberConstants.OUTER_RETRACTION_HEIGHT)) {
            switch (climberGoal) {

              case Override:
                break;

              case Idle:
                break;

              case ClimbL1:
                break;

              case Lock:
                break;

              default:
                climberState = ClimberState.L2Extend;
                break;
            }
          }
          break;

        case L2Extend:
          setInnerTargetHeight(ClimberConstants.L2_CLIMB_HEIGHT);
          if (isInnerClimberAtTargetHeight(ClimberConstants.L2_CLIMB_HEIGHT)) {
            climberState = ClimberState.L2Retract;
          }
          break;

        case L2Retract:
          setInnerTargetHeight(ClimberConstants.INNER_RETRACTION_HEIGHT);
          if (isInnerClimberAtTargetHeight(ClimberConstants.INNER_RETRACTION_HEIGHT)) {
            setInnerTargetHeight(0);
            switch (climberGoal) {

              case Override:
                break;

              case Idle:
                break;

              case Lock:
                break;

              case ClimbL1:
                break;

              case ClimbL2:
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
            climberState = ClimberState.L3Retract;
          }
          break;

        case L3Retract:
          setOuterTargetHeight(ClimberConstants.OUTER_RETRACTION_HEIGHT);
          break;

        default:
          break;
      }
    }
  }

  // setters

  public void setGoal(Goal goal) {
    climberGoal = goal;
    switch (goal) {

      case Override:
        climberState = ClimberState.Override;
        break;

      case Idle:
        climberState = ClimberState.Idle;
        break;

      case Lock:
        climberState = ClimberState.Lock;
        break;

      case MountDismount:
        climberState = ClimberState.MountDismount;
        break;

      case ClimbL1:
        climberState = ClimberState.MountDismount;
        break;

      case ClimbL2:
        climberState = ClimberState.MountDismount;
        break;

      case ClimbL3:
        climberState = ClimberState.MountDismount;
        break;

      default:
        break;
    }
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
