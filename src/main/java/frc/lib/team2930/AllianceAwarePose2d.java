package frc.lib.team2930;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;

public class AllianceAwarePose2d {
  private final Pose2d source;
  private Pose2d       target;
  private boolean      wasRed;

  public AllianceAwarePose2d(Pose2d source) {
    this.source = source;
  }

  public Pose2d get() {
    var currentlyRed = Constants.isRedAlliance();
    if (target == null || currentlyRed != wasRed) {
      wasRed = currentlyRed;
      target = currentlyRed ? GeometryUtil.rotatePose2DAroundCenterPoint(source) : source;
    }

    return target;
  }

  public Pose2d getAndFixRotation(boolean isisRobotRotatedBackward) {
    var pose = get();

    if (isisRobotRotatedBackward) {
      return GeometryUtil.faceBackward(pose);
    }

    return pose;
  }
}
