// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.Objects;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.lib.team2930.*;
import frc.lib.team6328.LoggedTunableNumber;

public class RotateToFaceHelper {
  private final String ROOT_TABLE = "RotateToFaceHelper";

  public static class Logs extends AutoRegisterLoggers {
    public LoggerEntry.Decimal rotVel;
    public LoggerEntry.Decimal targetRotation;
    public LoggerEntry.Decimal currentRotation;
    public LoggerEntry.Decimal positionErrorDeg;
  }

  public final Logs logs = LoggerGroup.root.newAutoLogger(Logs.class, ROOT_TABLE);

  public static class Tunables extends AutoRegisterTunables {
    @TunableEntry(defaultValue = 4.9)
    public LoggedTunableNumber KP;

    @TunableEntry(defaultValue = 2 * Math.PI)
    public LoggedTunableNumber cruiseVelocity;

    @TunableEntry(defaultValue = 4 * Math.PI)
    public LoggedTunableNumber maxAcceleration;

    @TunableEntry(defaultValue = 1)
    public LoggedTunableNumber toleranceDegrees;
  }

  public record RotateToFaceState(double omega, boolean withinTolerance, Distance distance) {
  }

  private final Tunables              tunables      = TunableNumberGroup.newAutoTunable(Tunables.class, ROOT_TABLE);
  private final ProfiledPIDController rotationalPID = new ProfiledPIDController(0, 0, 0, new Constraints(0.0, 0.0));

  private Translation2d target = new Translation2d();

  /** Creates a new RotateToFaceHelper . */
  public RotateToFaceHelper() {
  }

  public void resetPID(Pose2d currentPose, Pose2d fieldRelativeVelocities) {
    rotationalPID.reset(currentPose.getRotation().getRadians(), fieldRelativeVelocities.getRotation().getRadians());
    rotationalPID.setP(tunables.KP.get());
    rotationalPID.setConstraints(new Constraints(tunables.cruiseVelocity.get(), tunables.maxAcceleration.get()));
    rotationalPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  private static Rotation2d faceTowardsCenter(Translation2d current, Translation2d target,
      boolean isRobotRotatedBackward) {
    double targetX = target.getX();
    double targetY = target.getY();

    double robotX = current.getX();
    double robotY = current.getY();

    double angleToCenter = Math.atan2(targetY - robotY, targetX - robotX);

    double normalizedAngleToCenter = Math.atan2(Math.sin(angleToCenter), Math.cos(angleToCenter));

    Rotation2d finalRotation = new Rotation2d(normalizedAngleToCenter);

    if (isRobotRotatedBackward) {
      return finalRotation;
    }

    return finalRotation.rotateBy(Rotation2d.k180deg);
  }

  public RotateToFaceState periodic(Pose2d pose, boolean isRobotRotatedBackward) {
    Translation2d currentTranslation = pose.getTranslation();

    var targetangle         = RotateToFaceHelper.faceTowardsCenter(currentTranslation, target, isRobotRotatedBackward);
    var currentPoseRotation = pose.getRotation();
    var rotVel              = rotationalPID.calculate(currentPoseRotation.getRadians(), targetangle.getRadians());

    logs.rotVel.info(rotVel);
    logs.targetRotation.info(targetangle);
    logs.currentRotation.info(currentPoseRotation);
    logs.positionErrorDeg.info(Math.toDegrees(getError()));

    return new RotateToFaceState(rotVel, withinTolerance(), Units.Meters.of(target.getDistance(currentTranslation)));
  }

  public boolean setTarget(Translation2d target) {
    boolean reset = !Objects.equals(this.target, target);
    this.target = target;
    return reset;
  }

  private boolean withinTolerance() {
    return Math.abs(Math.toDegrees(getError())) <= tunables.toleranceDegrees.get();
  }

  private double getError() {
    return rotationalPID.getPositionError();
  }
}
