// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.team6328;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import frc.robot.Constants;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class PoseEstimator {
  private static final double historyLengthSecs = 2.0;

  //
  // We create a single-linked list, with a tail and a head.
  // We insert from the tail, always in timestamp order.
  // The head contains the initial pose in its 'finalPose' field.
  //
  private final PoseUpdate headPoseUpdate = new PoseUpdate(-Double.MAX_VALUE, null);
  private final PoseUpdate tailPoseUpdate = new PoseUpdate(Double.MAX_VALUE, null);
  private final double[]   q;
  private final long       tagsMask;

  private double visionCutoff;
  public int     rejectionCutoff;
  public int     rejectionInvalidTags;
  public int     rejectionNoDriveData;

  public PoseEstimator(double x, double y, double theta, int[] validTags) {
    q = new double[] { x * x, y * y, theta * theta };

    var tagsMask = 0L;
    for (int tag : validTags) {
      tagsMask |= 1L << tag;
    }
    this.tagsMask = tagsMask;

    resetPose(Constants.zeroPose2d, Double.NaN);
  }

  /** Returns the latest robot pose based on drive and vision data. */
  public Pose2d getLatestPose() {
    //
    // We always have two elements in the list.
    // The tail is always in the future, so the previous element is the latest real
    // pose.
    return this.tailPoseUpdate.previousUpdate.getFinalPose();
  }

  /** Returns the timestamp of the last vision update. */
  public double getVisionStaleness() {
    var lastUpdate       = this.tailPoseUpdate.previousUpdate;
    var lastVisionUpdate = lastUpdate;
    while (lastVisionUpdate.visionUpdates.isEmpty()) {
      PoseUpdate previousUpdate = lastVisionUpdate.previousUpdate;
      if (previousUpdate == null) {
        return 5;
      }

      lastVisionUpdate = previousUpdate;
    }

    return lastUpdate.timestamp - lastVisionUpdate.timestamp;
  }

  public Pose2d getPoseAtTime(double timestamp) {
    if (Double.isNaN(timestamp)) {
      return getLatestPose();
    }

    var poseUpdateAfter = tailPoseUpdate.findExactTimestampOrMoreRecent(timestamp);
    if (poseUpdateAfter.twist == null) {
      // This means that we have no drive data, ignore vision.
      return Constants.zeroPose2d;
    }

    var poseUpdateBefore = poseUpdateAfter.previousUpdate;

    // Create partial twists
    double timestampGap = poseUpdateAfter.timestamp - poseUpdateBefore.timestamp;

    double timestampMid = (timestamp - poseUpdateBefore.timestamp) / timestampGap;
    var    twist        = GeomUtil.multiplyTwist(poseUpdateAfter.twist, timestampMid);

    // Apply drive twist
    return poseUpdateBefore.getFinalPose().exp(twist);
  }

  /** Resets the odometry to a known pose. */
  public void resetPose(Pose2d pose, double visionCutoff) {
    //
    // Reset the list, setting the final pose at the head.
    //
    headPoseUpdate.finalPose      = pose;
    tailPoseUpdate.previousUpdate = headPoseUpdate;
    this.visionCutoff             = visionCutoff;
  }

  /** Records a new drive movement. */
  public void addDriveData(double timestamp, Twist2d twist) {
    var poseUpdate = new PoseUpdate(timestamp, twist);

    var existingPoseUpdate = tailPoseUpdate.findExactTimestampOrMoreRecent(timestamp);

    //
    // 'existingPoseUpdate' is always after timestamp, if not equal.
    // Good to insert new update before it.
    //
    poseUpdate.previousUpdate         = existingPoseUpdate.previousUpdate;
    existingPoseUpdate.previousUpdate = poseUpdate;

    // Since we changed the history up to 'existingPoseUpdate', invalidate the
    // computation of the
    // newer items.
    invalidateFinalPoseUpTo(existingPoseUpdate);
  }

  private void invalidateFinalPoseUpTo(PoseUpdate target) {
    var cursor          = tailPoseUpdate;
    var newestTimestamp = cursor.previousUpdate.timestamp;

    while (true) {
      var previous = cursor.previousUpdate;
      if (previous == headPoseUpdate) {
        return; // Reached end of list, exit
      }

      // At this point, we know that 'previous' is a valid entry.

      if (previous.timestamp + historyLengthSecs < newestTimestamp) {
        //
        // Found a sample older than the window we want to keep.
        // Prune the list, capping it at this entry.
        // Relink it to the head.
        //
        headPoseUpdate.finalPose = previous.getFinalPose();
        cursor.previousUpdate    = headPoseUpdate;
        return;
      }

      if (target != null) {
        // Keep invalidating the computed poses, until we hit the target.
        cursor.finalPose = null;

        // Found the oldest entry to invalidate, clear the flag.
        if (cursor == target) {
          target = null;
        }
      }

      cursor = previous;
    }
  }

  /** Records a new set of vision updates. */
  public void addVisionData(List<TimestampedVisionUpdate> visionData, boolean useLowestStdDev) {
    double       lowestStdDev             = Double.MAX_VALUE;
    double       lowestStdDevTimestamp    = 0;
    VisionUpdate lowestStdDevVisionUpdate = null;

    for (var timestampedVisionUpdate : visionData) {
      var timestamp = timestampedVisionUpdate.timestamp;
      if (Double.isFinite(visionCutoff) && timestamp < visionCutoff) {
        rejectionCutoff += 1;
        continue;
      }

      var tagsMask = 0L;
      for (var tag : timestampedVisionUpdate.tags) {
        tagsMask |= 1L << tag;
      }

      if ((this.tagsMask & tagsMask) != tagsMask) {
        rejectionInvalidTags += 1;
        continue;
      }

      var visionUpdate = timestampedVisionUpdate.update;

      if (useLowestStdDev) {
        var stddev = visionUpdate.StdDev();

        if (lowestStdDev > stddev) {
          lowestStdDev             = stddev;
          lowestStdDevVisionUpdate = visionUpdate;
          lowestStdDevTimestamp    = timestamp;
        }
      } else {
        acceptVisionUpdate(visionUpdate, timestamp);
      }
    }

    if (lowestStdDevVisionUpdate != null) {
      acceptVisionUpdate(lowestStdDevVisionUpdate, lowestStdDevTimestamp);
    }
  }

  private void acceptVisionUpdate(VisionUpdate visionUpdate, double timestamp) {
    var existingPoseUpdateAfter = tailPoseUpdate.findExactTimestampOrMoreRecent(timestamp);
    if (existingPoseUpdateAfter.twist == null)
    // This means that we have no drive data, ignore vision.
    {
      rejectionNoDriveData += 1;
      return;
    }

    if (existingPoseUpdateAfter.timestamp == timestamp) {
      // There was already an update at this timestamp, add to it
      var oldVisionUpdates = existingPoseUpdateAfter.visionUpdates;
      oldVisionUpdates.add(visionUpdate);
      oldVisionUpdates.sort(VisionUpdate.compareDescStdDev);
    } else {
      // Insert a new update
      var existingPoseUpdateBefore = existingPoseUpdateAfter.previousUpdate;

      // Create partial twists (prev -> vision, vision -> next)
      var timestampGap = existingPoseUpdateAfter.timestamp - existingPoseUpdateBefore.timestamp;
      var twist0       = GeomUtil.multiplyTwist(existingPoseUpdateAfter.twist,
          (timestamp - existingPoseUpdateBefore.timestamp) / timestampGap);

      var twist1 = GeomUtil.multiplyTwist(existingPoseUpdateAfter.twist,
          (existingPoseUpdateAfter.timestamp - timestamp) / timestampGap);

      // Add new pose updates
      var poseUpdate = new PoseUpdate(timestamp, twist0);
      poseUpdate.visionUpdates.add(visionUpdate);
      poseUpdate.previousUpdate = existingPoseUpdateBefore;

      // Update the post-timestamp twist.
      existingPoseUpdateAfter.previousUpdate = poseUpdate;
      existingPoseUpdateAfter.twist          = twist1;
    }

    // Since we changed the history, invalidate the computation of the newer items.
    invalidateFinalPoseUpTo(existingPoseUpdateAfter);
  }

  /**
   * Represents a sequential update to a pose estimate, with a twist (drive
   * movement) and list of vision updates.
   */
  class PoseUpdate {
    final double             timestamp;
    final List<VisionUpdate> visionUpdates = new ArrayList<>();

    Twist2d    twist;
    Pose2d     finalPose;
    PoseUpdate previousUpdate;

    PoseUpdate(double timestamp, Twist2d twist) {
      this.timestamp = timestamp;
      this.twist     = twist;
    }

    Pose2d getFinalPose() {
      if (this.finalPose == null) {
        var lastPose = this.previousUpdate.getFinalPose();

        // Apply drive twist
        var pose = lastPose.exp(twist);

        // Apply vision updates
        for (var visionUpdate : visionUpdates) {
          // Calculate Kalman gains based on std devs
          // (https://github.com/wpilibsuite/allwpilib/blob/main/wpimath/src/main/java/edu/wpi/first/math/estimator/)
          double[] confidence = new double[3];

          confidence[0] = updateKalmanComponent(q[0], visionUpdate.xStandardDeviation);
          confidence[1] = updateKalmanComponent(q[1], visionUpdate.yStandardDeviation);
          confidence[2] = updateKalmanComponent(q[2], visionUpdate.thetaStandardDeviation);

          // Calculate twist between current and vision pose
          var visionTwist = pose.log(visionUpdate.pose());

          // Apply twist, multipling by Kalman gain values
          pose = pose.exp(new Twist2d(visionTwist.dx * confidence[0], visionTwist.dy * confidence[1],
              visionTwist.dtheta * confidence[2]));
        }

        // Cache result of computation.
        this.finalPose = pose;
      }

      return this.finalPose;
    }

    static double updateKalmanComponent(double v, double stdDev) {
      if (v == 0.0) {
        return 0;
      }

      return v / (v + Math.sqrt(v * stdDev * stdDev));
    }

    PoseUpdate findExactTimestampOrMoreRecent(double timestamp) {
      var cursor = this;
      while (true) {
        var previous = cursor.previousUpdate;
        if (previous.timestamp <= timestamp) {
          return cursor;
        }

        cursor = previous;
      }
    }
  }

  /** Represents a single vision pose with associated standard deviations. */
  public record VisionUpdate(Pose2d pose, double xStandardDeviation, double yStandardDeviation,
      double thetaStandardDeviation) {

    public static final Comparator<VisionUpdate> compareDescStdDev = (VisionUpdate a, VisionUpdate b) -> {
      return -Double.compare(a.xStandardDeviation + a.yStandardDeviation + a.yStandardDeviation,
          b.xStandardDeviation + b.yStandardDeviation + b.thetaStandardDeviation);
    };

    public double StdDev() {
      return xStandardDeviation * xStandardDeviation + yStandardDeviation * yStandardDeviation
          + thetaStandardDeviation * thetaStandardDeviation;
    }
  }

  /**
   * Represents a single vision pose with a timestamp and associated standard
   * deviations.
   */
  public record TimestampedVisionUpdate(List<Short> tags, double timestamp, VisionUpdate update) {
  }
}
