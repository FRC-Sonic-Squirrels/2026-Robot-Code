package frc.robot.autonomous.helpers;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import frc.lib.team2930.*;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.autonomous.records.ChoreoTrajectoryWithName;

import java.util.ArrayList;
import java.util.List;

public class ChoreoHelper {
  private final String ROOT_TABLE = "ChoreoHelper";

  public static class Logs extends AutoRegisterLoggers {
    public LoggerEntry.Decimal             stateLinearVel;
    public LoggerEntry.Decimal             stateLinearVelError;
    public LoggerEntry.Decimal             stateScaleVel;
    public LoggerEntry.Decimal             stateTimeOffset;
    public LoggerEntry.Decimal             stateTimestamp;
    public LoggerEntry.Struct<Pose2d>      closestPose;
    public LoggerEntry.Struct<Pose2d>      optimalPose;
    public LoggerEntry.Struct<Pose2d>      desiredVelocity;
    public LoggerEntry.Decimal             distanceError;
    public LoggerEntry.Decimal             offsetError;
    public LoggerEntry.Decimal             targetError;
    public LoggerEntry.Decimal             headingError;
    public LoggerEntry.Decimal             pidXVelEffort;
    public LoggerEntry.Decimal             pidYVelEffort;
    public LoggerEntry.Decimal             pidVelEffort;
    public LoggerEntry.StructArray<Pose2d> path;
    public LoggerEntry.Text                stallReason;
    public LoggerEntry.Bool                isPaused;
  }

  public final Logs logs = LoggerGroup.root.newAutoLogger(Logs.class, ROOT_TABLE);

  public static class Tunables extends AutoRegisterTunables {
    @TunableEntry(defaultValue = 1.0)
    public LoggedTunableNumber useCorrection;

    @TunableEntry(defaultValue = 2_0)
    public LoggedTunableNumber minVelToPause;
  }

  private final Tunables tunables = TunableNumberGroup.newAutoTunable(Tunables.class, ROOT_TABLE);

  private final Trajectory<SwerveSample> traj;
  private final PIDController            xFeedback;
  private final PIDController            yFeedback;
  private final PIDController            rotationalFeedback;
  private final double                   initialTime;
  private final double                   lagThreshold;

  private double       timeOffset;
  private double       pausedTime = Double.NaN;
  private SwerveSample stateTooBehind;

  private double finalOffsetErrorLimit  = Double.NaN;
  private double finalTargetErrorLimit  = Double.NaN;
  private double finalHeadingErrorLimit = Double.NaN;
  private double finalErrorMaxWait      = 2;
  private double finalErrorWaitDeadline = Double.NaN;

  public record ChassisSpeedsWithPathEnd(ChassisSpeeds chassisSpeeds, boolean atEndOfPath) {
  }

  /**
   * Helper class to go from timestamps of path to desired chassis speeds
   *
   * @param initialTime            timestamp of path starting
   * @param initialPose            initial pose of robot
   * @param trajWithName           trajectory to follow
   * @param lagThreshold           distance meters before pausing path (for PID to
   *                               catch up)
   * @param translationalFeedbackX pid in x directions
   * @param translationalFeedbackY pid in y directions
   * @param rotationalFeedback     pid for angular velocity
   */
  public ChoreoHelper(double initialTime, Pose2d initialPose, ChoreoTrajectoryWithName trajWithName,
      double lagThreshold, PIDController translationalFeedbackX, PIDController translationalFeedbackY,
      PIDController rotationalFeedback) {
    this.traj               = trajWithName.states();
    this.lagThreshold       = lagThreshold;
    this.xFeedback          = translationalFeedbackX;
    this.yFeedback          = translationalFeedbackY;
    this.rotationalFeedback = rotationalFeedback;
    this.rotationalFeedback.enableContinuousInput(-Math.PI, Math.PI);

    SwerveSample closestState    = null;
    double       closestDistance = Double.MAX_VALUE;
    double       lastDistance    = Double.MAX_VALUE;

    if (tunables.useCorrection.get() != 0) {
      for (SwerveSample state : getStates(traj)) {
        SwerveSample stateComputed = traj.sampleAt(state.t, false).orElseThrow();
        double       stateDistance = GeometryUtil.getDist(initialPose, stateComputed.getPose());

        if (stateDistance > lastDistance) {
          // Moving away, give up.
          break;
        }

        if (closestState == null || stateDistance < closestDistance) {
          closestState    = state;
          closestDistance = stateDistance;
        }

        lastDistance = stateDistance;
      }
    }

    if (closestState != null) {
      this.timeOffset = closestState.t;
      logs.closestPose.info(closestState.getPose());
    }

    this.initialTime = initialTime;
    var poses = trajWithName.states().getPoses();
    logs.path.info(poses);
  }

  public void setFinalOffsetError(double maxError) {
    this.finalOffsetErrorLimit = Math.abs(maxError);
  }

  public void setFinalTargetError(double maxError) {
    this.finalTargetErrorLimit = Math.abs(maxError);
  }

  public void setFinalHeadingError(double maxError) {
    this.finalHeadingErrorLimit = Math.abs(maxError);
  }

  public void setFinalErrorMaxWait(double maxWait) {
    this.finalErrorMaxWait = Math.abs(maxWait);
  }

  public boolean isPaused() {
    return Double.isFinite(pausedTime);
  }

  public void pause(double timestamp) {
    if (!isPaused()) {
      pausedTime = timestamp;
    }
  }

  public void resume(double timestamp) {
    if ((isPaused())) {
      timeOffset += (pausedTime - timestamp);
      pausedTime  = Double.NaN;
    }
  }

  /**
   * Calculates field relative chassis speeds from path
   *
   * @param robotPose pose of the robot
   * @param timestamp time of path
   */
  public ChassisSpeedsWithPathEnd calculateChassisSpeeds(Pose2d robotPose, double timestamp) {
    SwerveSample state;

    logs.isPaused.info(isPaused());

    boolean atTheEndOfPath = false;

    if (stateTooBehind != null) {
      state = stateTooBehind;
    } else {
      var timestampCorrected = timestamp - initialTime + timeOffset;

      state = traj.sampleAt(timestampCorrected, false).orElseThrow();
      if (timestampCorrected >= traj.getTotalTime()) {
        atTheEndOfPath = true;
      }
    }

    while (true) {
      var lookaheadTime = Constants.kDefaultPeriod;

      var stateAhead = isFutureStateCloser(robotPose, state, lookaheadTime);
      if (stateAhead == null) {
        break;
      }

      timeOffset += lookaheadTime;
      state       = stateAhead;
      if (stateTooBehind != null) {
        stateTooBehind = state;
      }
    }

    logs.stateTimestamp.info(state.t);
    logs.stateTimeOffset.info(timeOffset);

    double xRobot = robotPose.getX();
    double yRobot = robotPose.getY();

    double xDesired = state.x;
    double yDesired = state.y;

    var distanceError = Math.hypot(xDesired - xRobot, yDesired - yRobot);

    var statePose        = new Pose2d(state.x, state.y, new Rotation2d(state.heading));
    var robotToStatePose = robotPose.relativeTo(statePose);

    double targetError = robotToStatePose.getX();
    double offsetError = robotToStatePose.getY();
    logs.distanceError.info(distanceError);
    logs.targetError.info(targetError);
    logs.offsetError.info(offsetError);

    boolean useCorrection = tunables.useCorrection.get() != 0;
    if (useCorrection) {
      if (distanceError < lagThreshold) {
        if (stateTooBehind != null) {
          stateTooBehind = null;
          resume(timestamp);
        }
      } else {
        if (stateTooBehind == null) {
          var velMagnitude = Math.hypot(state.vx, state.vy);
          if (velMagnitude >= tunables.minVelToPause.get() && state.t > 0.25) {
            stateTooBehind = state;
            pause(timestamp);
          }
        }
      }
    }

    double scaleVelocity = 1.0;

    if (Double.isFinite(pausedTime)) {
      if (stateTooBehind == null) {
        atTheEndOfPath = true;
      }

      // If we are paused, scale down the velocity, to avoid fighting the Feedback
      // PID.
      scaleVelocity *= Math.exp(-(timestamp - pausedTime));
    }

    logs.stateScaleVel.info(scaleVelocity);

    double pidXVel = xFeedback.calculate(xRobot, xDesired);
    double pidYVel = yFeedback.calculate(yRobot, yDesired);

    double xVel = state.vx * scaleVelocity;
    double yVel = state.vy * scaleVelocity;

    if (useCorrection) {
      xVel += pidXVel;
      yVel += pidYVel;
    }

    logs.pidXVelEffort.info(pidXVel);
    logs.pidYVelEffort.info(pidYVel);
    logs.pidVelEffort.info(Math.hypot(pidXVel, pidYVel));
    logs.stateLinearVel.info(Math.hypot(state.vx, state.vy));
    logs.stateLinearVelError.info(Math.hypot(state.vx - xVel, state.vy - yVel));

    Rotation2d rotation     = robotPose.getRotation();
    double     theta        = rotation.getRadians();
    double     omegaVel     = state.omega + rotationalFeedback.calculate(theta, state.heading);
    double     headingError = Math.toDegrees(GeometryUtil.optimizeRotation(theta - state.heading));

    String waitingOn = "";

    if (atTheEndOfPath) {
      var waitForOffset  = Double.isFinite(finalOffsetErrorLimit);
      var waitForTarget  = Double.isFinite(finalTargetErrorLimit);
      var waitForHeading = Double.isFinite(finalHeadingErrorLimit);

      if (waitForOffset || waitForTarget || waitForHeading) {
        if (!Double.isFinite(finalErrorWaitDeadline)) {
          // Start timer, we don't want to get stuck here forever.
          finalErrorWaitDeadline = timestamp + finalErrorMaxWait;
        }

        // Only delay end of path if timer has not fired.
        if (timestamp < finalErrorWaitDeadline) {
          if (waitForOffset && Math.abs(offsetError) > finalOffsetErrorLimit) {
            waitingOn      = (waitingOn + " Offset").trim();
            atTheEndOfPath = false;
          }

          if (waitForTarget && Math.abs(targetError) > finalTargetErrorLimit) {
            waitingOn      = (waitingOn + " Target").trim();
            atTheEndOfPath = false;
          }

          if (waitForOffset && Math.abs(headingError) > finalHeadingErrorLimit) {
            waitingOn      = (waitingOn + " Heading").trim();
            atTheEndOfPath = false;
          }
        }
      }
    }

    logs.optimalPose.info(state.getPose());
    logs.desiredVelocity.info(new Pose2d(xVel, yVel, Rotation2d.fromRadians(omegaVel)));
    logs.headingError.info(headingError);
    logs.stallReason.info(waitingOn);

    var chassisSpeeds = new ChassisSpeeds(xVel, yVel, omegaVel);
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, rotation);
    return new ChassisSpeedsWithPathEnd(chassisSpeeds, atTheEndOfPath);
  }

  private SwerveSample isFutureStateCloser(Pose2d robotPose, SwerveSample state, double lookaheadTime) {
    var stateAhead = traj.sampleAt(state.t + lookaheadTime, false).orElseThrow();

    double stateDistance      = distanceToState(robotPose, state);
    double stateDistanceAhead = distanceToState(robotPose, stateAhead);
    return stateDistanceAhead < stateDistance ? stateAhead : null;
  }

  private static double distanceToState(Pose2d robotPose, SwerveSample state) {
    double xRobot = robotPose.getX();
    double yRobot = robotPose.getY();

    double xDesired = state.x;
    double yDesired = state.y;

    return Math.hypot(xDesired - xRobot, yDesired - yRobot);
  }

  @SuppressWarnings({
      "rawtypes",
      "unchecked"
  })
  public static Trajectory<SwerveSample> rescale(Trajectory<SwerveSample> traj, double speedScaling) {
    if (speedScaling == 1.0) {
      return traj;
    }

    var newStates = new ArrayList<SwerveSample>();
    for (var state : getStates(traj)) {
      newStates.add(new SwerveSample(state.t * speedScaling, state.x, state.y, state.heading, state.vx / speedScaling,
          state.vy / speedScaling, state.omega / speedScaling, state.ax / speedScaling, state.ay / speedScaling,
          state.alpha / speedScaling, state.moduleForcesX(), state.moduleForcesY()));
    }

    return new Trajectory(traj.name(), newStates, traj.splits(), traj.events());
  }

  @SuppressWarnings({
      "rawtypes",
      "unchecked"
  })
  public static Trajectory<SwerveSample> flipOnAlliance(Trajectory<SwerveSample> states) {
    var newStates = new ArrayList<SwerveSample>();

    for (var state : getStates(states)) {
      newStates.add(new SwerveSample(state.t, state.x, Constants.FieldConstants.FIELD_WIDTH.in(Units.Meter) - state.y,
          -state.heading, state.vx, -state.vy, -state.omega, state.ax, -state.ay, -state.alpha, state.moduleForcesX(),
          state.moduleForcesY()));
    }

    return new Trajectory(states.name(), newStates, states.splits(), states.events());
  }

  @SuppressWarnings("unchecked")
  private static List<SwerveSample> getStates(Trajectory<SwerveSample> traj) {
    try {
      var f = traj.getClass().getDeclaredField("samples");
      f.setAccessible(true);
      // noinspection unchecked
      return (List<SwerveSample>) f.get(traj);
    } catch (NoSuchFieldException | IllegalAccessException e) {
      throw new RuntimeException(e);
    }
  }
}
