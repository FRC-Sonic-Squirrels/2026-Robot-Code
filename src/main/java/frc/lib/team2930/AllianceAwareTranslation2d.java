package frc.lib.team2930;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public class AllianceAwareTranslation2d {
  private final Translation2d source;
  private Translation2d       target;
  private boolean             wasRed;

  public AllianceAwareTranslation2d(Translation2d source) {
    this.source = source;
  }

  public Translation2d get() {
    var currentlyRed = Constants.isRedAlliance();
    if (target == null || currentlyRed != wasRed) {
      wasRed = currentlyRed;
      target = currentlyRed ? GeometryUtil.rotateTranslation2dAroundCenterPoint(source) : source;
    }

    return target;
  }

  public Pose2d getPose2d() {
    return new Pose2d(get(), Rotation2d.kZero);
  }
}
