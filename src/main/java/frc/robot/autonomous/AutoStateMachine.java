package frc.robot.autonomous;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.team2930.StateMachine;
import frc.robot.autonomous.AutosManager.Waypoint;
import frc.robot.commands.drive.DriveToPoseHelper;
import frc.robot.subsystems.swerve.DrivetrainWrapper;

public class AutoStateMachine extends StateMachine {
  private final DrivetrainWrapper   wrapper;
  private final DriveToPoseHelper   driveHelper;
  private final ArrayList<Waypoint> waypoints;
  private int                       waypointCounter;

  public AutoStateMachine(AutosSubsystems subsystems, ArrayList<Waypoint> waypoints, Pose2d initialPose) {
    super("Auto");
    this.wrapper    = subsystems.drivetrain();
    this.waypoints  = waypoints;
    driveHelper     = new DriveToPoseHelper(() -> false);
    waypointCounter = 0;

    driveHelper.resetPID(initialPose, wrapper.getFieldRelativeVelocities());
    setInitialState(this::nextState);
  }

  private StateHandler driveToPose() {

    wrapper.setVelocityOverride(driveHelper.periodic(wrapper.getHubPoseEstimatorPose(true)));

    if (!driveHelper.isAtTarget()) {
      return null;
    }

    return nextState();
  }

  private StateHandler nextState() {
    if (waypointCounter >= waypoints.size()) {
      return setDone();
    }

    Waypoint nextWaypoint = waypoints.get(waypointCounter);
    driveHelper.setTarget(nextWaypoint.waypointPose());
    waypointCounter++;
    switch (nextWaypoint.goal()) {
      case DRIVE:
        break;
      case CLIMB:
        break;
      case NOTHING:
        break;
      case INTAKEWHILEDRIVE:
        break;
      case SCORE:
        break;
      default:
        break;
    }

    return stateWithName("DriveToPose", this::driveToPose);
  }

}
