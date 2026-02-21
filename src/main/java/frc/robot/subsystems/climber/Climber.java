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

  private enum State {
    // Leader is the outer elevator, follower is the inner elevator.
    // Retract is elevator going down, pulling the robot up.
    // Extend is elevator going up, reaching for the next bar.
    //@formatter:off
    Idle,
    LeaderMountDismount,
    L1LeaderRetract,
    L1FollowerExtend,
    L1FollowerRetract,
    L2LeaderExtend,
    L2LeaderRetract,
    L2FollowerExtend,
    L2FollowerRetract,
    L3LeaderExtend,
    L3LeaderRetract,
    Lock
    //@formatter:on
  }

  public static class Logs extends AutoRegisterLoggers {
    public LoggerEntry.EnumValue<State> state;
    public LoggerEntry.EnumValue<Goal>  goal;
    public LoggerEntry.Decimal          servoTargetAngle;
  }

  private Servo climberServo = new Servo(ClimberConstants.SERVO_CHANNEL);

  public final Logs logs = LoggerGroup.root.newAutoLogger(Logs.class, ROOT_TABLE);

  // Execution timing
  private final ExecutionTiming climberTiming = new ExecutionTiming(ROOT_TABLE);

  private final Elevator innerElevator;
  private final Elevator outerElevator;

  private State state = State.Idle;
  private Goal  goal  = Goal.Idle;

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

        case Idle:
          switch (state) {

            case Idle:
              setLeaderTargetHeight(ClimberConstants.LEADER_RETRACTED_HEIGHT);
              setFollowerTargetHeight(ClimberConstants.FOLLOWER_RETRACTED_HEIGHT);
              break;

            case LeaderMountDismount:
              state = State.Idle;
              setLeaderTargetHeight(ClimberConstants.LEADER_RETRACTED_HEIGHT);
              setFollowerTargetHeight(ClimberConstants.FOLLOWER_RETRACTED_HEIGHT);
              break;

            default:
              break;
          }
          break;

        case Lock:// FIXME get info about the lock and servo systems
          switch (state) {

            case Idle:
              break;

            case Lock:
              break;

            case LeaderMountDismount:
              break;

            case L1LeaderRetract:
              state = State.Lock; // Generic lock code for if the lock is on the leader
              setServoAngle(ClimberConstants.SERVO_LOCK_ANGLE);
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
              break;
          }
          break;

        case MountDismount:
          switch (state) {

            case Idle:
              state = State.LeaderMountDismount;
              setLeaderTargetHeight(ClimberConstants.L1_LEADER_CLIMB_HEIGHT);
              break;

            default:
              break;
          }
          break;

        case ClimbL1:
          switch (state) {

            case LeaderMountDismount:
              state = State.L1LeaderRetract;
              setLeaderTargetHeight(ClimberConstants.LEADER_RETRACTED_HEIGHT);
              break;

            case L1LeaderRetract:
              if (isLeaderAtTargetHeight(ClimberConstants.LEADER_RETRACTED_HEIGHT)) {
                state = State.L1FollowerExtend;
                setFollowerTargetHeight(ClimberConstants.FOLLOWER_EXTENDED_HEIGHT);
              }
              break;

            case L1FollowerExtend:
              if (isFollowerAtTargetHeight(ClimberConstants.FOLLOWER_EXTENDED_HEIGHT)) {
                state = State.L1FollowerRetract;
                setFollowerTargetHeight(ClimberConstants.FOLLOWER_RETRACTED_HEIGHT);
              }
              break;

            default:
              break;
          }
          break;

        case ClimbL2:
          switch (state) {

            case LeaderMountDismount:
              state = State.L1LeaderRetract;
              setLeaderTargetHeight(ClimberConstants.LEADER_RETRACTED_HEIGHT);
              break;

            case L1LeaderRetract:
              if (isLeaderAtTargetHeight(ClimberConstants.LEADER_RETRACTED_HEIGHT)) {
                state = State.L1FollowerExtend;
                setFollowerTargetHeight(ClimberConstants.FOLLOWER_EXTENDED_HEIGHT);
              }
              break;

            case L1FollowerExtend:
              if (isFollowerAtTargetHeight(ClimberConstants.FOLLOWER_EXTENDED_HEIGHT)) {
                state = State.L1FollowerRetract;
                setFollowerTargetHeight(ClimberConstants.FOLLOWER_RETRACTED_HEIGHT);
              }
              break;

            case L1FollowerRetract:
              if (isFollowerAtTargetHeight(ClimberConstants.FOLLOWER_RETRACTED_HEIGHT)) {
                state = State.L2LeaderExtend;
                setLeaderTargetHeight(ClimberConstants.L2_LEADER_CLIMB_HEIGHT);
              }
              break;

            case L2LeaderExtend:
              if (isLeaderAtTargetHeight(ClimberConstants.L2_LEADER_CLIMB_HEIGHT)) {
                state = State.L2LeaderRetract;
                setLeaderTargetHeight(ClimberConstants.LEADER_RETRACTED_HEIGHT);
              }
              break;

            case L2LeaderRetract:
              if (isLeaderAtTargetHeight(ClimberConstants.LEADER_RETRACTED_HEIGHT)) {
                state = State.L2FollowerExtend;
                setFollowerTargetHeight(ClimberConstants.FOLLOWER_EXTENDED_HEIGHT);
              }
              break;

            case L2FollowerExtend:
              if (isFollowerAtTargetHeight(ClimberConstants.FOLLOWER_EXTENDED_HEIGHT)) {
                state = State.L2FollowerRetract;
                setFollowerTargetHeight(ClimberConstants.FOLLOWER_RETRACTED_HEIGHT);
              }
              break;

            default:
              break;
          }
          break;

        case ClimbL3:
          switch (state) {

            case LeaderMountDismount:
              state = State.L1LeaderRetract;
              setLeaderTargetHeight(ClimberConstants.LEADER_RETRACTED_HEIGHT);
              break;

            case L1LeaderRetract:
              if (isLeaderAtTargetHeight(ClimberConstants.LEADER_RETRACTED_HEIGHT)) {
                state = State.L1FollowerExtend;
                setFollowerTargetHeight(ClimberConstants.FOLLOWER_EXTENDED_HEIGHT);
              }
              break;

            case L1FollowerExtend:
              if (isFollowerAtTargetHeight(ClimberConstants.FOLLOWER_EXTENDED_HEIGHT)) {
                state = State.L1FollowerRetract;
                setFollowerTargetHeight(ClimberConstants.FOLLOWER_RETRACTED_HEIGHT);
              }
              break;

            case L1FollowerRetract:
              if (isFollowerAtTargetHeight(ClimberConstants.FOLLOWER_RETRACTED_HEIGHT)) {
                state = State.L2LeaderExtend;
                setLeaderTargetHeight(ClimberConstants.L2_LEADER_CLIMB_HEIGHT);
              }
              break;

            case L2LeaderExtend:
              if (isFollowerAtTargetHeight(ClimberConstants.L2_LEADER_CLIMB_HEIGHT)) {
                state = State.L2LeaderRetract;
                setLeaderTargetHeight(ClimberConstants.LEADER_RETRACTED_HEIGHT);
              }
              break;

            case L2LeaderRetract:
              if (isLeaderAtTargetHeight(ClimberConstants.LEADER_RETRACTED_HEIGHT)) {
                state = State.L2FollowerExtend;
                setFollowerTargetHeight(ClimberConstants.FOLLOWER_EXTENDED_HEIGHT);
              }
              break;

            case L2FollowerExtend:
              if (isFollowerAtTargetHeight(ClimberConstants.FOLLOWER_EXTENDED_HEIGHT)) {
                state = State.L2FollowerRetract;
                setFollowerTargetHeight(ClimberConstants.FOLLOWER_RETRACTED_HEIGHT);
              }
              break;

            case L2FollowerRetract:
              if (isFollowerAtTargetHeight(ClimberConstants.FOLLOWER_RETRACTED_HEIGHT)) {
                state = State.L3LeaderExtend;
                setLeaderTargetHeight(ClimberConstants.L3_LEADER_CLIMB_HEIGHT);
              }
              break;

            case L3LeaderExtend:
              if (isLeaderAtTargetHeight(ClimberConstants.L3_LEADER_CLIMB_HEIGHT)) {
                state = State.L3LeaderRetract;
                setLeaderTargetHeight(ClimberConstants.LEADER_RETRACTED_HEIGHT);
              }
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
