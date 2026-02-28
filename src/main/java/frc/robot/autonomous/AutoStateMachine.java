package frc.robot.autonomous;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.team2930.StateMachine;
import frc.robot.autonomous.AutosManager.Waypoint;
import frc.robot.commands.drive.DriveToPoseHelper;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.DrivetrainWrapper;

public class AutoStateMachine extends StateMachine {
  private final DrivetrainWrapper   wrapper;
  private final LED                 led;
  private final Intake              leftIntake;
  private final Intake              rightIntake;
  private final DriveToPoseHelper   driveHelper;
  private final ArrayList<Waypoint> waypoints;
  private Pose2d                    waypointTargetPose;
  private int                       waypointCounter;

  public AutoStateMachine(AutosSubsystems subsystems, ArrayList<Waypoint> waypoints, Pose2d initialPose) {
    super("Auto");
    this.wrapper       = subsystems.drivetrain();
    this.led           = subsystems.led();
    this.leftIntake    = subsystems.leftIntake();
    this.rightIntake   = subsystems.rightIntake();
    this.waypoints     = waypoints;
    driveHelper        = new DriveToPoseHelper(() -> false);
    waypointCounter    = 0;
    waypointTargetPose = waypoints.get(waypointCounter).waypointPose();

    driveHelper.resetPID(initialPose, wrapper.getFieldRelativeVelocities());
    setInitialState(() -> driveToPose());
  }

  private StateHandler driveToPose() {
    driveHelper.setTarget(waypointTargetPose);
    // while (!waypointTargetPose.equals(wrapper.getHubPoseEstimatorPose(true))) {
    wrapper.setVelocityOverride(driveHelper.periodic(wrapper.getHubPoseEstimatorPose(true)));

    // }

    return nextState();
  }

  private StateHandler nextState() {
    if (waypointCounter >= waypoints.size()) {
      return null;
    }
    Waypoint tempWaypoint = waypoints.get(waypointCounter);
    switch (tempWaypoint.goal()) {
      case DRIVE:
        waypointCounter++;
        waypointTargetPose = waypoints.get(waypointCounter).waypointPose();
        return stateWithName("DriveToPose", () -> driveToPose());
      case CLIMB:
        return null;
      case NOTHING:
        return null;
      case INTAKEWHILEDRIVE:
        return null;
      case SCORE:
        return null;
      default:
        return null;
    }
  }

}
