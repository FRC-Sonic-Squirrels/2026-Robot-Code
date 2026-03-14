package frc.lib.team2930;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public class AllianceAwareVelocity2d {
  private final Translation2d source;
  private Translation2d       target;
  private boolean             wasRed;

  public AllianceAwareVelocity2d(Translation2d source) {
    this.source = source;
  }

  public Translation2d get() {
    var currentlyRed = Constants.isRedAlliance();
    if (target == null || currentlyRed != wasRed) {
      wasRed = currentlyRed;
      target = currentlyRed ? new Translation2d(-source.getX(), -source.getY()) : source;
    }

    return target;
  }

  public Pose2d getPose2d() {
    return new Pose2d(get(), Rotation2d.kZero);
  }
}
