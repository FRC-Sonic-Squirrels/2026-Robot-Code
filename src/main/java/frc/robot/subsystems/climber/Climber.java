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
    Idle, Override, L1Retract, MountDismount, L2Extend, L2Retract, L3Extend, L3Retract, Lock
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
          state = State.Override;
          break;

        case Idle:
          switch (state) {

            case Override:
              break;

            case Idle:
              setInnerTargetHeight(ClimberConstants.INNER_RETRACTION_HEIGHT);
              setOuterTargetHeight(ClimberConstants.OUTER_RETRACTION_HEIGHT);
              break;

            case Lock:
              break;

            case MountDismount:
              state = State.Idle;
              break;

            case L1Retract:
              break;

            case L2Extend:
              break;

            case L2Retract:
              break;

            case L3Extend:
              break;

            case L3Retract:
              break;
          }
          break;

        case Lock:
          switch (state) {

            case Override:
              break;

            case Idle:
              break;

            case Lock:
              break;

            case MountDismount:
              break;

            case L1Retract:
              state = State.Lock;
              setServoAngle(ClimberConstants.SERVO_LOCK_ANGLE);
              break;

            case L2Extend:
              break;

            case L2Retract:
              state = State.Lock;
              setServoAngle(ClimberConstants.SERVO_LOCK_ANGLE);
              break;

            case L3Extend:
              break;

            case L3Retract:
              state = State.Lock;
              setServoAngle(ClimberConstants.SERVO_LOCK_ANGLE);
              break;
          }
          break;

        case MountDismount:
          switch (state) {

            case Override:
              break;

            case Idle:
              state = State.MountDismount;
              setOuterTargetHeight(ClimberConstants.L1_CLIMB_HEIGHT);
              break;

            case Lock:
              break;

            case MountDismount:
              break;

            case L1Retract:
              state = State.MountDismount;
              setOuterTargetHeight(ClimberConstants.L1_CLIMB_HEIGHT);
              break;

            case L2Extend:
              break;

            case L2Retract:
              break;

            case L3Extend:
              break;

            case L3Retract:
              break;
          }
          break;

        case ClimbL1:
          switch (state) {

            case Override:
              break;

            case Idle:
              break;

            case Lock:
              break;

            case MountDismount:
              state = State.L1Retract;
              setOuterTargetHeight(ClimberConstants.OUTER_RETRACTION_HEIGHT);
              break;

            case L1Retract:
              break;

            case L2Extend:
              break;

            case L2Retract:
              break;

            case L3Extend:
              break;

            case L3Retract:
              break;
          }
          break;

        case ClimbL2:
          switch (state) {

            case Override:
              break;

            case Idle:
              break;

            case Lock:
              break;

            case MountDismount:
              state = State.L1Retract;
              setOuterTargetHeight(ClimberConstants.OUTER_RETRACTION_HEIGHT);
              break;

            case L1Retract:
              if (isOuterClimberAtTargetHeight(ClimberConstants.OUTER_RETRACTION_HEIGHT)) {
                state = State.L2Extend;
                setInnerTargetHeight(ClimberConstants.L2_CLIMB_HEIGHT);
              }
              break;

            case L2Extend:
              if (isInnerClimberAtTargetHeight(ClimberConstants.L2_CLIMB_HEIGHT)) {
                state = State.L2Retract;
                setInnerTargetHeight(ClimberConstants.INNER_RETRACTION_HEIGHT);
              }
              break;

            case L2Retract:
              break;

            case L3Extend:
              break;

            case L3Retract:
              break;
          }
          break;

        case ClimbL3:
          switch (state) {

            case Override:
              break;

            case Idle:
              break;

            case Lock:
              break;

            case MountDismount:
              state = State.L1Retract;
              setOuterTargetHeight(ClimberConstants.OUTER_RETRACTION_HEIGHT);
              break;

            case L1Retract:
              if (isOuterClimberAtTargetHeight(ClimberConstants.OUTER_RETRACTION_HEIGHT)) {
                state = State.L2Extend;
                setInnerTargetHeight(ClimberConstants.L2_CLIMB_HEIGHT);
              }
              break;

            case L2Extend:
              if (isInnerClimberAtTargetHeight(ClimberConstants.L2_CLIMB_HEIGHT)) {
                state = State.L2Retract;
                setInnerTargetHeight(ClimberConstants.INNER_RETRACTION_HEIGHT);
              }
              break;

            case L2Retract:
              if (isInnerClimberAtTargetHeight(ClimberConstants.INNER_RETRACTION_HEIGHT)) {
                state = State.L3Extend;
                setOuterTargetHeight(ClimberConstants.L3_CLIMB_HEIGHT);
              }
              break;

            case L3Extend:
              if (isOuterClimberAtTargetHeight(ClimberConstants.L3_CLIMB_HEIGHT)) {
                state = State.L3Retract;
                setOuterTargetHeight(ClimberConstants.OUTER_RETRACTION_HEIGHT);
              }
              break;

            case L3Retract:
              break;
          }
          break;

        default:
          break;
      }
    }
  }

  // setters

  public void setGoal(Goal goal) {
    this.goal = goal;
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
