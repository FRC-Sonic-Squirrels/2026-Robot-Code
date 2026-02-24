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
    MountDismount, ClimbL1, ClimbL2, ClimbL3, Stowed, Override, Lock
  }

  private enum State {
    // Leader is the outer elevator, follower is the inner elevator.
    // Retract is elevator going down, pulling the robot up.
    // Extend is elevator going up, reaching for the next bar.
    //@formatter:off
    Stowed           (ClimberConstants.LEADER_HEIGHT_STOWED, ClimberConstants.FOLLOWER_HEIGHT_STOWED),
    L1LeaderExtend   (ClimberConstants.LEADER_HEIGHT_CLIMBL1,  ClimberConstants.FOLLOWER_HEIGHT_RETRACTED),
    L1LeaderRetract  (ClimberConstants.LEADER_HEIGHT_RETRACTED, ClimberConstants.FOLLOWER_HEIGHT_RETRACTED),
    L1FollowerExtend (ClimberConstants.LEADER_HEIGHT_RETRACTED, ClimberConstants.FOLLOWER_HEIGHT_EXTENDED),
    L1FollowerRetract(ClimberConstants.LEADER_HEIGHT_RETRACTED, ClimberConstants.FOLLOWER_HEIGHT_RETRACTED),
    L2LeaderExtend   (ClimberConstants.LEADER_HEIGHT_CLIMBL2,  ClimberConstants.FOLLOWER_HEIGHT_RETRACTED),
    L2LeaderRetract  (ClimberConstants.LEADER_HEIGHT_RETRACTED, ClimberConstants.FOLLOWER_HEIGHT_RETRACTED),
    L2FollowerExtend (ClimberConstants.LEADER_HEIGHT_RETRACTED, ClimberConstants.FOLLOWER_HEIGHT_EXTENDED),
    L2FollowerRetract(ClimberConstants.LEADER_HEIGHT_RETRACTED, ClimberConstants.FOLLOWER_HEIGHT_RETRACTED),
    L3LeaderExtend   (ClimberConstants.LEADER_HEIGHT_CLIMBL3,  ClimberConstants.FOLLOWER_HEIGHT_RETRACTED),
    L3LeaderRetract  (ClimberConstants.LEADER_HEIGHT_RETRACTED, ClimberConstants.FOLLOWER_HEIGHT_RETRACTED),
    L3FollowerExtend (ClimberConstants.LEADER_HEIGHT_RETRACTED, ClimberConstants.FOLLOWER_HEIGHT_EXTENDED),
    L3FollowerRetract(ClimberConstants.LEADER_HEIGHT_RETRACTED, ClimberConstants.FOLLOWER_HEIGHT_RETRACTED),
    Lock             (ClimberConstants.LEADER_HEIGHT_RETRACTED, ClimberConstants.FOLLOWER_HEIGHT_RETRACTED);
    //@formatter:on

    public final double leaderHeight;
    public final double followerHeight;

    State(double leaderHeight, double followerHeight) {
      this.leaderHeight   = leaderHeight;
      this.followerHeight = followerHeight;
    }
  }

  public static class Logs extends AutoRegisterLoggers {
    public LoggerEntry.EnumValue<State> state;
    public LoggerEntry.EnumValue<Goal>  goal;
    public LoggerEntry.Decimal          servoTargetAngle;
  }

  private Servo climberServo = new Servo(ClimberConstants.SERVO_CHANNEL);

  public final Logs logs   = LoggerGroup.root.newAutoLogger(Logs.class, ROOT_TABLE);
  public boolean    atGoal = false;

  // Execution timing
  private final ExecutionTiming climberTiming = new ExecutionTiming(ROOT_TABLE);

  private final Elevator innerElevator;
  private final Elevator outerElevator;

  private State state = State.Stowed;
  private Goal  goal  = Goal.Stowed;

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

      logs.state.info(state);
      logs.goal.info(goal);
      logs.servoTargetAngle.info(servoTargetAngle);

      switch (goal) {

        case Override:
          break;

        case Stowed:
          switch (state) {

            case L1LeaderExtend:
              handleTransitions(State.Stowed);
              atGoal = true;
              break;

            default:
              break;
          }
          break;

        case Lock:// FIXME get info about the lock and servo systems
          switch (state) {

            case Stowed:
              break;

            case Lock:
              break;

            case L1LeaderExtend:
              break;

            case L1LeaderRetract:
              state = State.Lock; // Generic lock code for if the lock is on the leader
              setServoAngle(ClimberConstants.SERVO_LOCK_ANGLE);
              atGoal = true;
              break;

            case L1FollowerExtend:
              break;

            case L1FollowerRetract:
              break;

            case L2LeaderExtend:
              break;

            case L2LeaderRetract:
              state = State.Lock; // Generic lock code for if the lock is on the leader
              setServoAngle(ClimberConstants.SERVO_LOCK_ANGLE);
              atGoal = true;
              break;

            case L2FollowerExtend:
              break;

            case L2FollowerRetract:
              break;

            case L3LeaderExtend:
              break;

            case L3LeaderRetract:
              state = State.Lock; // Generic lock code for if the lock is on the leader
              setServoAngle(ClimberConstants.SERVO_LOCK_ANGLE);
              atGoal = true;
              break;

            case L3FollowerExtend:
              break;

            case L3FollowerRetract:
              break;
          }
          break;

        case MountDismount:
          switch (state) {

            case Stowed:
              handleTransitions(State.L1LeaderExtend);
              break;

            case L1LeaderExtend:
              atGoal = true;
              break;

            case L1LeaderRetract:
              handleTransitions(State.L1LeaderExtend);
              break;

            default:
              break;
          }
          break;

        case ClimbL1:
          switch (state) {

            case L1LeaderExtend:
              handleTransitions(State.L1LeaderRetract);
              break;

            case L1LeaderRetract:
              atGoal = true;
              break;

            default:
              break;
          }
          break;

        case ClimbL2:
          switch (state) {

            case L1LeaderExtend:
              handleTransitions(State.L1LeaderRetract);
              break;

            case L1LeaderRetract:
              handleTransitions(State.L1FollowerExtend);
              break;

            case L1FollowerExtend:
              handleTransitions(State.L1FollowerRetract);
              break;

            case L1FollowerRetract:
              handleTransitions(State.L2LeaderExtend);
              break;

            case L2LeaderExtend:
              handleTransitions(State.L2LeaderRetract);
              break;

            case L2LeaderRetract:
              handleTransitions(State.L2FollowerExtend);
              break;

            case L2FollowerExtend:
              handleTransitions(State.L2FollowerRetract);
              break;

            case L2FollowerRetract:
              atGoal = true;
              break;

            default:
              break;
          }
          break;

        case ClimbL3:
          switch (state) {

            case L1LeaderExtend:
              handleTransitions(State.L1LeaderRetract);
              break;

            case L1LeaderRetract:
              handleTransitions(State.L1FollowerExtend);
              break;

            case L1FollowerExtend:
              handleTransitions(State.L1FollowerRetract);
              break;

            case L1FollowerRetract:
              handleTransitions(State.L2LeaderExtend);
              break;

            case L2LeaderExtend:
              handleTransitions(State.L2LeaderRetract);
              break;

            case L2LeaderRetract:
              handleTransitions(State.L2FollowerExtend);
              break;

            case L2FollowerExtend:
              handleTransitions(State.L2FollowerRetract);
              break;

            case L2FollowerRetract:
              handleTransitions(State.L3LeaderExtend);
              break;

            case L3LeaderExtend:
              handleTransitions(State.L3LeaderRetract);
              break;

            case L3LeaderRetract:
              handleTransitions(State.L3FollowerExtend);
              break;

            case L3FollowerExtend:
              handleTransitions(State.L3FollowerRetract);
              break;

            case L3FollowerRetract:
              atGoal = true;
              break;

            default:
              break;
          }
          break;
      }
    }
  }

  // setters

  public void setGoal(Goal goal) {
    this.goal = goal;
  }

  public void setFollowerTargetHeight(double height) {
    innerElevator.setTargetHeight(height);
  }

  public void setLeaderTargetHeight(double height) {
    outerElevator.setTargetHeight(height);
  }

  private void handleTransitions(State desiredState) {
    atGoal = false;
    setLeaderTargetHeight(desiredState.leaderHeight);
    setFollowerTargetHeight(desiredState.followerHeight);
    if (isLeaderAtTargetHeight(desiredState.leaderHeight) && isFollowerAtTargetHeight(desiredState.followerHeight)) {
      state = desiredState;
    }
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

  public boolean isFollowerAtTargetHeight(double target) {
    return innerElevator.isElevatorAtTargetHeight(target);
  }

  public boolean isLeaderAtTargetHeight(double target) {
    return outerElevator.isElevatorAtTargetHeight(target);
  }
}
