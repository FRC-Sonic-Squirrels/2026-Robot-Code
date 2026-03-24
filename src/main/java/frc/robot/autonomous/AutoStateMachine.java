package frc.robot.autonomous;

import choreo.trajectory.EventMarker;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.team2930.AllianceAwarePose2d;
import frc.lib.team2930.StateMachine;
import frc.robot.Constants;
import frc.robot.RobotStates;
import frc.robot.RobotStates.IntakeGoal;
import frc.robot.RobotStates.RobotGoal;
import frc.robot.autonomous.AutosManager.Waypoint;
import frc.robot.autonomous.helpers.ChoreoHelper;
import frc.robot.autonomous.records.ChoreoTrajectoryWithName;
import frc.robot.subsystems.swerve.DrivetrainWrapper;

import java.util.ArrayList;
import java.util.List;
import java.util.TreeMap;

public class AutoStateMachine extends StateMachine {
  private final DrivetrainWrapper         wrapper;
  private final boolean                   skipAhead;
  private final AllianceAwarePose2d       initialPose;
  private final ArrayList<Waypoint>       waypoints;
  private final RobotStates               states;
  private ChoreoHelper                    helper;
  private int                             currentWaypoint = -1;
  private final TreeMap<Double, Waypoint> samplesMap      = new TreeMap<>();

  public AutoStateMachine(AutosSubsystems subsystems, ArrayList<Waypoint> waypoints, AllianceAwarePose2d initialPose,
      boolean skipAhead, RobotStates states) {
    super("Auto");

    this.wrapper     = subsystems.drivetrain();
    this.skipAhead   = skipAhead;
    this.initialPose = initialPose;
    this.waypoints   = waypoints;
    this.states      = states;

    setInitialState(this::initializeChoreo);
  }

  private StateHandler initializeChoreo() {
    var config = states.config;

    List<SwerveSample> samples = new ArrayList<>();
    List<Integer>      splits  = new ArrayList<>();
    List<EventMarker>  events  = new ArrayList<>();

    double time = 0;

    samplesMap.clear();
    int sequence = 0;

    for (Waypoint waypoint : waypoints) {
      waypoint.sequence = sequence++;
      var waypointTime = waypoint.time;
      var waypointPose = waypoint.pose;
      var waypointVel  = waypoint.velocity;

      time += waypointTime;

      if (waypointPose != null) {
        var pose     = waypointPose.get();
        var velocity = waypointVel != null ? waypointVel.get() : Translation2d.kZero;

        var swerveSample = new SwerveSample(time, pose.getX(), pose.getY(), pose.getRotation().getRadians(),
            velocity.getX(), velocity.getY(), 0, 0, 0, 0, null, null);
        samples.add(swerveSample);
      }

      samplesMap.put(time, waypoint);
    }

    helper = new ChoreoHelper(skipAhead, Timer.getFPGATimestamp(), initialPose.get(),
        new ChoreoTrajectoryWithName("Auto", new Trajectory<>("Auto", samples, splits, events)),
        config.getDriveBaseRadius() / 2, config.getAutoTranslationPidController(),
        config.getAutoTranslationPidController(), config.getAutoThetaPidController());

    return this::driveToPose;
  }

  private StateHandler driveToPose() {
    ChoreoHelper.ChassisSpeedsWithPathEnd result = helper.calculateChassisSpeeds(wrapper.getHubPoseEstimatorPose(true),
        Timer.getFPGATimestamp());

    wrapper.setVelocityOverride(result.chassisSpeeds());

    if (Constants.unusedCode) {
      System.out.println("Time: " + result.timeOffset() + " " + result.currentState().t);
    }
    var waypoint = matchWaypoint(result);
    if (waypoint != null && waypoint.sequence != currentWaypoint) {
      if (Constants.unusedCode) {
        System.out.println("Waypoint: " + waypoint.sequence + " # " + waypoint.goal);
      }

      currentWaypoint = waypoint.sequence;

      switch (waypoint.goal) {
        case DRIVE:
        case CLIMB:
        case NOTHING:
          states.setRobotGoal(RobotGoal.None);
          states.setIntakeGoal(IntakeGoal.Stow);
          break;

        case INTAKEWHILEDRIVELEFT:
          states.setRobotGoal(RobotGoal.None);
          states.setIntakeGoal(IntakeGoal.LeftIntake);
          break;

        case INTAKEWHILEDRIVERIGHT:
          states.setRobotGoal(RobotGoal.None);
          states.setIntakeGoal(IntakeGoal.RightIntake);
          break;

        case SCORE:
          states.setRobotGoal(RobotGoal.Shoot);
          break;
      }
    }

    if (!result.atEndOfPath()) {
      return null;
    }

    wrapper.setVelocityOverride(null);
    return setDone();
  }

  private Waypoint matchWaypoint(ChoreoHelper.ChassisSpeedsWithPathEnd state) {
    var entry = samplesMap.floorEntry(state.timeOffset());
    if (entry != null) {
      return entry.getValue();
    }

    return null;
  }
}
