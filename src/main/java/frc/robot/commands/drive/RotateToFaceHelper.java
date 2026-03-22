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
    public LoggerEntry.Decimal             rotVel;
    public LoggerEntry.Decimal             targetRotation;
    public LoggerEntry.Decimal             currentRotation;
    public LoggerEntry.Decimal             positionErrorDeg;
    public LoggerEntry.StructArray<Pose2d> path;

    public LoggerEntry.Struct<Pose2d> correctedPose;
  }

  public final Logs logs = LoggerGroup.root.newAutoLogger(Logs.class, ROOT_TABLE);

  public static class Tunables extends AutoRegisterTunables {
    @TunableEntry(defaultValue = 4.9)
    public LoggedTunableNumber KP;

    @TunableEntry(defaultValue = 0.0)
    public LoggedTunableNumber KI;

    @TunableEntry(defaultValue = 0.0)
    public LoggedTunableNumber KD;

    @TunableEntry(defaultValue = 0.1)
    public LoggedTunableNumber moveCorrection;

    @TunableEntry(defaultValue = 2 * Math.PI)
    public LoggedTunableNumber cruiseVelocity;

    @TunableEntry(defaultValue = 4 * Math.PI)
    public LoggedTunableNumber maxAcceleration;

    @TunableEntry(defaultValue = 4)
    public LoggedTunableNumber toleranceDegrees;

    public boolean hasChanged() {
      boolean hasChanged = false;
      hasChanged |= KP.hasChanged();
      hasChanged |= KI.hasChanged();
      hasChanged |= KD.hasChanged();
      hasChanged |= moveCorrection.hasChanged();
      hasChanged |= cruiseVelocity.hasChanged();
      hasChanged |= maxAcceleration.hasChanged();
      hasChanged |= toleranceDegrees.hasChanged();
      return hasChanged;
    }
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
    rotationalPID.enableContinuousInput(-Math.PI, Math.PI);
    refreshTunables();
  }

  private void refreshTunables() {
    rotationalPID.setConstraints(new Constraints(tunables.cruiseVelocity.get(), tunables.maxAcceleration.get()));
    rotationalPID.setP(tunables.KP.get());
    rotationalPID.setI(tunables.KI.get());
    rotationalPID.setD(tunables.KD.get());
  }

  private static Rotation2d faceTowardsCenter(Translation2d current, Translation2d target) {
    double targetX = target.getX();
    double targetY = target.getY();

    double robotX = current.getX();
    double robotY = current.getY();

    double angleToCenter = Math.atan2(targetY - robotY, targetX - robotX);

    double normalizedAngleToCenter = Math.atan2(Math.sin(angleToCenter), Math.cos(angleToCenter));

    return new Rotation2d(normalizedAngleToCenter);
  }

  public RotateToFaceState periodic(Pose2d pose, Pose2d robotVelocity) {
    if (tunables.hasChanged()) {
      refreshTunables();
    }

    var currentPose         = pose.getTranslation();
    var currentPoseRotation = pose.getRotation();
    var currentVel          = robotVelocity.getTranslation();
    var correctedVel        = currentVel.times(tunables.moveCorrection.get());

    // We have to add the future movement to get the correct angle.
    var correctedPose = currentPose.plus(correctedVel);
    var targetAngle   = RotateToFaceHelper.faceTowardsCenter(correctedPose, target);

    var rotVel          = rotationalPID.calculate(currentPoseRotation.getRadians(), targetAngle.getRadians());
    var error           = Math.toDegrees(rotationalPID.getPositionError());
    var withinTolerance = Math.abs(error) <= tunables.toleranceDegrees.get();

    logs.correctedPose.info(new Pose2d(correctedPose, currentPoseRotation));

    logs.rotVel.info(rotVel);
    logs.targetRotation.info(targetAngle);
    logs.currentRotation.info(currentPoseRotation);
    logs.positionErrorDeg.info(error);

    var beam = new Translation2d(2, 0).rotateBy(currentPoseRotation);

    Pose2d[] path = new Pose2d[2];
    path[0] = new Pose2d(correctedPose, Rotation2d.kZero);
    path[1] = new Pose2d(correctedPose.plus(beam), Rotation2d.kZero);
    logs.path.info(path);

    return new RotateToFaceState(rotVel, withinTolerance, Units.Meters.of(target.getDistance(correctedPose)));
  }

  public boolean setTarget(Translation2d target) {
    boolean reset = !Objects.equals(this.target, target);
    this.target = target;
    return reset;
  }
}
