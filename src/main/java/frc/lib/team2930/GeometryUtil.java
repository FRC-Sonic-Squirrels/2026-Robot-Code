package frc.lib.team2930;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;

public class GeometryUtil {
  public static Rotation2d getHeading(Translation2d pose, Translation2d targetPose) {
    if (pose.getNorm() == 0 && targetPose.getNorm() == 0) {
      return Rotation2d.kZero;
    }
    return new Rotation2d(targetPose.getX() - pose.getX(), targetPose.getY() - pose.getY());
  }

  public static boolean isRotationAtTarget(Rotation2d angle, Rotation2d targetAngle, double toleranceInDegrees) {
    var error = angle.minus(targetAngle).getDegrees();
    return Math.abs(error) <= toleranceInDegrees;
  }

  public static boolean isRotationAtTarget(Rotation2d angle, Rotation2d targetAngle, Rotation2d tolerance) {
    return isRotationAtTarget(angle, targetAngle, tolerance.getDegrees());
  }

  public static double getDist(Translation2d pose, Translation2d pose2) {
    return Math.hypot(pose.getX() - pose2.getX(), pose.getY() - pose2.getY());
  }

  public static double getDist(Pose2d pose, Pose2d pose2) {
    return Math.hypot(pose.getX() - pose2.getX(), pose.getY() - pose2.getY());
  }

  public static Rotation2d newRotation2d(double x, double y) {
    return Math.abs(x) > 0.01 || Math.abs(y) > 0.01 ? new Rotation2d(x, y) : Rotation2d.kZero;
  }

  public static Translation3d translation2dTo3d(Translation2d translation) {
    return new Translation3d(translation.getX(), translation.getY(), 0.0);
  }

  public static Rotation3d rotation2dTo3d(Rotation2d rotation) {
    return new Rotation3d(0.0, 0.0, rotation.getRadians());
  }

  public static Translation3d translation3dFromMeasures(Distance x, Distance y, Distance z) {
    return new Translation3d(x.in(Units.Meters), y.in(Units.Meters), z.in(Units.Meters));
  }

  public static boolean isPoseOutsideField(Pose2d pose) {
    double poseX = pose.getX();
    double poseY = pose.getY();
    if (poseX < 0.0 || poseY < 0.0 || poseX > 16.534 || poseY > 8.21) {
      return true;
    }
    return false;
  }

  public static double optimizeRotation(double theta) {
    while (theta <= -Math.PI) {
      theta += Math.PI * 2;
    }
    while (theta >= Math.PI) {
      theta -= Math.PI * 2;
    }
    return theta;
  }

  public static double optimizeRotationInDegrees(double theta) {
    while (theta < -180) {
      theta += 360;
    }
    while (theta > 180) {
      theta -= 360;
    }
    return theta;
  }

  public static Pose3d rotatePose3dAroundTranslation2d(Pose3d initialPose, Translation2d referenceTranslation,
      Rotation2d rot) {
    Translation2d offset            = initialPose.getTranslation().toTranslation2d().minus(referenceTranslation);
    Pose3d        zeroedPose        = new Pose3d(offset.getX(), offset.getY(), initialPose.getZ(),
        initialPose.getRotation());
    Pose3d        rotatedZeroedPose = zeroedPose.rotateBy(new Rotation3d(rot));
    return new Pose3d(rotatedZeroedPose.getX() + initialPose.getX() - offset.getX(),
        rotatedZeroedPose.getY() + initialPose.getY() - offset.getY(), rotatedZeroedPose.getZ(),
        rotatedZeroedPose.getRotation());
  }

  public static Translation2d rotateTranslation2dAroundCenterPoint(Translation2d originalTranslation) {
    return new Translation2d(Constants.FieldConstants.FIELD_LENGTH_METERS - originalTranslation.getX(),
        Constants.FieldConstants.FIELD_WIDTH_METERS - originalTranslation.getY());
  }

  public static Pose2d rotatePose2DAroundCenterPoint(Pose2d originalPose) {
    return new Pose2d(rotateTranslation2dAroundCenterPoint(originalPose.getTranslation()),
        originalPose.getRotation().plus(Rotation2d.k180deg));
  }

  public static Pose3d rotatePose3DAroundCenterPoint(Pose3d originalPose) {
    Pose2d internalPose2d        = originalPose.toPose2d();
    Pose2d flippedInternalPose2d = rotatePose2DAroundCenterPoint(internalPose2d);
    return new Pose3d(flippedInternalPose2d.getX(), flippedInternalPose2d.getY(), originalPose.getZ(),
        new Rotation3d(originalPose.getRotation().getX(), originalPose.getRotation().getY(),
            flippedInternalPose2d.getRotation().getRadians()));
  }

  public static Translation2d getClosestTranslation(Translation2d[] selection, Pose2d currentPose) {
    Translation2d lowestDistTranslation = null;
    double        lowestDist            = Double.MAX_VALUE;
    for (Translation2d translation : selection) {
      double distance = currentPose.getTranslation().getDistance(translation);
      if (distance < lowestDist) {
        lowestDistTranslation = translation;
        lowestDist            = distance;
      }
    }
    for (int i = 0; i < selection.length; i++) {
    }
    return lowestDistTranslation;
  }

  public static Translation2d getClosestTranslation(AllianceAwareTranslation2d[] selection, Pose2d currentPose) {
    Translation2d[] normArr = new Translation2d[selection.length];
    for (int i = 0; i < selection.length; i++) {
      normArr[i] = selection[i].get();
    }
    return getClosestTranslation(normArr, currentPose);
  }

  public static Pose2d getClosestPose(Pose2d[] selection, Pose2d currentPose) {
    Pose2d lowestDistPose = null;
    double lowestDist     = Double.MAX_VALUE;
    for (Pose2d pose : selection) {
      double distance = currentPose.getTranslation().getDistance(pose.getTranslation());
      if (distance < lowestDist) {
        lowestDistPose = pose;
        lowestDist     = distance;
      }
    }
    return lowestDistPose;
  }

  public static Pose2d getClosestPose(AllianceAwarePose2d[] selection, Pose2d currentPose) {
    Pose2d[] normArr = new Pose2d[selection.length];
    for (int i = 0; i < selection.length; i++) {
      normArr[i] = selection[i].get();
    }
    return getClosestPose(normArr, currentPose);
  }
}
