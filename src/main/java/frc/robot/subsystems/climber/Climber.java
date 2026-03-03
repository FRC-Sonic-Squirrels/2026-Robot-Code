package frc.robot.subsystems.climber;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
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
    Stowed           (ClimberConstants.LEADER_HEIGHT_STOWED   , ClimberConstants.FOLLOWER_HEIGHT_STOWED   ),
    L1LeaderExtend   (ClimberConstants.LEADER_HEIGHT_CLIMBL1  , ClimberConstants.FOLLOWER_HEIGHT_STOWED   ),
    L1LeaderRetract  (ClimberConstants.LEADER_HEIGHT_RETRACTED, ClimberConstants.FOLLOWER_HEIGHT_STOWED   ),
    L1FollowerExtend (ClimberConstants.LEADER_HEIGHT_RETRACTED, ClimberConstants.FOLLOWER_HEIGHT_EXTENDED ),
    L1FollowerRetract(ClimberConstants.LEADER_HEIGHT_RETRACTED, ClimberConstants.FOLLOWER_HEIGHT_RETRACTED),
    L2LeaderExtend   (ClimberConstants.LEADER_HEIGHT_CLIMBL2  , ClimberConstants.FOLLOWER_HEIGHT_RETRACTED),
    L2LeaderRetract  (ClimberConstants.LEADER_HEIGHT_RETRACTED, ClimberConstants.FOLLOWER_HEIGHT_RETRACTED),
    L2FollowerExtend (ClimberConstants.LEADER_HEIGHT_RETRACTED, ClimberConstants.FOLLOWER_HEIGHT_EXTENDED ),
    L2FollowerRetract(ClimberConstants.LEADER_HEIGHT_RETRACTED, ClimberConstants.FOLLOWER_HEIGHT_RETRACTED),
    L3LeaderExtend   (ClimberConstants.LEADER_HEIGHT_CLIMBL3  , ClimberConstants.FOLLOWER_HEIGHT_RETRACTED),
    L3LeaderRetract  (ClimberConstants.LEADER_HEIGHT_RETRACTED, ClimberConstants.FOLLOWER_HEIGHT_RETRACTED),
    Lock             (ClimberConstants.LEADER_HEIGHT_RETRACTED, ClimberConstants.FOLLOWER_HEIGHT_RETRACTED);
    //@formatter:on

    public final Distance leaderHeight;
    public final Distance followerHeight;

    State(Distance leaderHeight, Distance followerHeight) {
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

  private final Elevator leader;
  private final Elevator follower;

  private State state = State.Stowed;
  private Goal  goal  = Goal.Stowed;

  // This also acts as its current angle because the servo cannot tell us where it
  // is
  private Rotation2d servoTargetAngle = Constants.ClimberConstants.SERVO_LOCK_ANGLE;

  /** Creates a new ClimberSubsystem. */
  public Climber(ElevatorIO leaderIO, ElevatorIO followerIO) {
    this.leader   = null;                                                     // new
                                                                              // Elevator(logs.group.subgroup("Leader"),
                                                                              // leaderIO);
    this.follower = new Elevator(logs.group.subgroup("Follower"), followerIO);

    setServoAngle(ClimberConstants.SERVO_UNLOCK_ANGLE);
  }

  @Override
  public void periodic() {
    leader.periodic();
    follower.periodic();
    try (var ignored = climberTiming.start()) {
      // Climber logging

      logs.state.info(state);
      logs.goal.info(goal);
      logs.servoTargetAngle.info(servoTargetAngle);
      boolean atGoalNext;
      switch (goal) {
        case Override:
          atGoalNext = true;
          break;

        case Stowed:
          switch (state) {
            case L1LeaderExtend:
              handleTransitions(State.Stowed);
              atGoalNext = false;
              break;

            case Stowed:
              handleTransitions(State.Stowed);
              atGoalNext = true;
              break;

            default:
              atGoalNext = false;
              break;
          }
          break;

        case Lock:
          switch (state) {
            case Lock:
              atGoalNext = true;
              break;

            case L1LeaderRetract:
            case L2LeaderRetract:
            case L3LeaderRetract:
              state = State.Lock; // Generic lock code for if the lock is on the leader
              setServoAngle(ClimberConstants.SERVO_LOCK_ANGLE);
              atGoalNext = true;
              break;

            default:
              atGoalNext = false;
              break;
          }
          break;

        case MountDismount:
          switch (state) {
            case Stowed:
              handleTransitions(State.L1LeaderExtend);
              atGoalNext = false;
              break;

            case L1LeaderExtend:
              atGoalNext = true;
              break;

            case L1LeaderRetract:
              handleTransitions(State.L1LeaderExtend);
              atGoalNext = false;
              break;

            default:
              atGoalNext = false;
              break;
          }
          break;

        case ClimbL1:
          switch (state) {
            case L1LeaderExtend:
              handleTransitions(State.L1LeaderRetract);
              atGoalNext = false;
              break;

            case L1LeaderRetract:
              atGoalNext = true;
              break;

            default:
              atGoalNext = false;
              break;
          }
          break;

        case ClimbL2:
          if (follower == null) {
            atGoalNext = false;
            break;
          }
          switch (state) {
            case L1LeaderExtend:
              handleTransitions(State.L1LeaderRetract);
              atGoalNext = false;
              break;

            case L1LeaderRetract:
              handleTransitions(State.L1FollowerExtend);
              atGoalNext = false;
              break;

            case L1FollowerExtend:
              handleTransitions(State.L1FollowerRetract);
              atGoalNext = false;
              break;

            case L1FollowerRetract:
              handleTransitions(State.L2LeaderExtend);
              atGoalNext = false;
              break;

            case L2LeaderExtend:
              handleTransitions(State.L2LeaderRetract);
              atGoalNext = false;
              break;

            case L2LeaderRetract:
              atGoalNext = true;
              break;

            default:
              atGoalNext = false;
              break;
          }
          break;

        case ClimbL3:
          if (follower == null) {
            atGoalNext = false;
            break;
          }
          switch (state) {
            case L1LeaderExtend:
              handleTransitions(State.L1LeaderRetract);
              atGoalNext = false;
              break;

            case L1LeaderRetract:
              handleTransitions(State.L1FollowerExtend);
              atGoalNext = false;
              break;

            case L1FollowerExtend:
              handleTransitions(State.L1FollowerRetract);
              atGoalNext = false;
              break;

            case L1FollowerRetract:
              handleTransitions(State.L2LeaderExtend);
              atGoalNext = false;
              break;

            case L2LeaderExtend:
              handleTransitions(State.L2LeaderRetract);
              atGoalNext = false;
              break;

            case L2LeaderRetract:
              handleTransitions(State.L2FollowerExtend);
              atGoalNext = false;
              break;

            case L2FollowerExtend:
              handleTransitions(State.L2FollowerRetract);
              atGoalNext = false;
              break;

            case L2FollowerRetract:
              handleTransitions(State.L3LeaderExtend);
              atGoalNext = false;
              break;

            case L3LeaderExtend:
              handleTransitions(State.L3LeaderRetract);
              atGoalNext = false;
              break;

            case L3LeaderRetract:
              atGoalNext = true;
              break;

            default:
              atGoalNext = false;
              break;
          }
          break;

        default:
          atGoalNext = false;
          break;
      }
      this.atGoal = atGoalNext;
    }
  }

  // setters

  public void setGoal(Goal goal) {
    this.goal = goal;
  }

  public void setFollowerTargetHeight(Distance height) {
    follower.setHeight(height);
  }

  public void setLeaderTargetHeight(Distance height) {
    leader.setHeight(height);
  }

  private void handleTransitions(State desiredState) {
    setLeaderTargetHeight(desiredState.leaderHeight);
    setFollowerTargetHeight(desiredState.followerHeight);
    if (isLeaderAtTargetHeight(desiredState.leaderHeight) && isFollowerAtTargetHeight(desiredState.followerHeight)) {
      state = desiredState;
    }
  }

  public boolean setClimberNeutralMode(NeutralModeValue value) {
    if (goal == Goal.Override) {
      return false;
    }
    return leader.setNeutralMode(value) && follower.setNeutralMode(value);
  }

  public void setServoAngle(Rotation2d angle) {
    climberServo.setAngle(angle.getDegrees());
    servoTargetAngle = angle;
  }

  public Rotation2d getServoAngle() {
    return servoTargetAngle;
  }

  // getters

  public boolean isFollowerAtTargetHeight(Distance target) {
    return follower.isAtTarget(target);
  }

  public boolean isLeaderAtTargetHeight(Distance target) {
    return leader.isAtTarget(target);
  }
}
