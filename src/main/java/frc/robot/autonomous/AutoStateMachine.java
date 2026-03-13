package frc.robot.autonomous;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.team2930.AllianceAwarePose2d;
import frc.lib.team2930.StateMachine;
import frc.robot.RobotStates;
import frc.robot.RobotStates.IntakeGoal;
import frc.robot.RobotStates.RobotGoal;
import frc.robot.autonomous.AutosManager.Waypoint;
import frc.robot.commands.drive.DriveToPoseHelper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.DrivetrainWrapper;

public class AutoStateMachine extends StateMachine {
  private final DrivetrainWrapper   wrapper;
  private final Intake leftIntake;
  private final Intake rightIntake;
  private final Shooter shooter;
  private final DriveToPoseHelper   driveHelper;
  private final ArrayList<Waypoint> waypoints;
  private int                       waypointCounter;
  private final RobotStates states;

  public AutoStateMachine(AutosSubsystems subsystems, ArrayList<Waypoint> waypoints, AllianceAwarePose2d initialPose, RobotStates states) {
    super("Auto");
    this.wrapper    = subsystems.drivetrain();
    this.leftIntake = subsystems.leftIntake();
    this.rightIntake = subsystems.rightIntake();
    this.shooter = subsystems.shooter();
    this.waypoints  = waypoints;
    driveHelper     = new DriveToPoseHelper(this::goSlow);
    this.states = states;
    waypointCounter = 0;

    driveHelper.resetPID(initialPose.get(), wrapper.getFieldRelativeVelocities());
    setInitialState(this::nextState);
  }

  private StateHandler driveToPose() {

    wrapper.setVelocityOverride(driveHelper.periodic(wrapper.getHubPoseEstimatorPose(true)));

    if (!driveHelper.isAtTarget()) {
      return null;
    }

    return nextState();
  }

  private StateHandler driveToPoseIntakingLeft(){

    states.setIntakeGoal(IntakeGoal.LeftIntake);
    //TODO: Slow down at some point
    wrapper.setVelocityOverride(driveHelper.periodic(wrapper.getHubPoseEstimatorPose(true)));

    if (!driveHelper.isAtTarget()) {
      return null;
    }

    return nextState();
  }

  private StateHandler driveToPoseIntakingRight(){
    states.setIntakeGoal(IntakeGoal.RightIntake);
    //TODO: Slow down at some point
    wrapper.setVelocityOverride(driveHelper.periodic(wrapper.getHubPoseEstimatorPose(true)));

    if (!driveHelper.isAtTarget()) {
      return null;
    }

    return nextState();
  }

  private StateHandler shoot(){
    var time = this.timeFromStartOfState();
    states.setRobotGoal(RobotGoal.Shoot);
    if((time <= 2)){
      return null;
    }

    states.setRobotGoal(RobotGoal.None);
    return nextState();
  }

  private StateHandler nextState() {
    if (waypointCounter >= waypoints.size()) {
      return setDone();
    }

    Waypoint nextWaypoint = waypoints.get(waypointCounter);
    driveHelper.setTarget(nextWaypoint.waypointPose().get());
    waypointCounter++;
    switch (nextWaypoint.goal()) {
      case DRIVE:
        break;
      case CLIMB:
        break;
      case NOTHING:
        break;
      case INTAKEWHILEDRIVELEFT:
        return stateWithName("DriveToPoseIntakeLeft",this::driveToPoseIntakingLeft);
      case INTAKEWHILEDRIVERIGHT:
        return stateWithName("DriveToPoseIntakeRight", this::driveToPoseIntakingRight);  
      case SCORE:
        
        return stateWithName("Shoot", this::shoot);
      default:
        break;
    }

    return stateWithName("DriveToPose", this::driveToPose);
  }

  private boolean goSlow(){
    if (waypointCounter >= waypoints.size()) {
      return false;
    }

    Waypoint nextWaypoint = waypoints.get(waypointCounter);
    return nextWaypoint.goSlow();
  }

}
