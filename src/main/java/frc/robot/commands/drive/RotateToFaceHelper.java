// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.lib.team2930.*;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import java.util.function.Supplier;

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

  private final Tunables tunables = TunableNumberGroup.newAutoTunable(Tunables.class, ROOT_TABLE);

  private final DrivetrainWrapper       wrapper;
  private final Supplier<Translation2d> target;
  private final Supplier<Pose2d>        currentPose;
  private final ProfiledPIDController   rotationalPID = new ProfiledPIDController(0, 0, 0, new Constraints(0.0, 0.0));

  private double currentDist = Double.MAX_VALUE;

  /** Creates a new RotateToFaceHelper . */
  public RotateToFaceHelper(DrivetrainWrapper wrapper, Supplier<Translation2d> target, Supplier<Pose2d> pose2d) {
    this.wrapper     = wrapper;
    this.target      = target;
    this.currentPose = pose2d;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public void initialize() {
    rotationalPID.reset(currentPose.get().getRotation().getRadians(),
        wrapper.getFieldRelativeVelocities().getRotation().getRadians());
    rotationalPID.setP(tunables.KP.get());
    rotationalPID.setConstraints(new Constraints(tunables.cruiseVelocity.get(), tunables.maxAcceleration.get()));
    rotationalPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  private static Rotation2d faceTowardsCenter(Translation2d current, Translation2d target) {

    double targetX = target.getX();
    double targetY = target.getY();

    double robotX = current.getX();
    double robotY = current.getY();

    double angleToCenter = Math.atan2(targetY - robotY, targetX - robotX);

    double normalizedAngleToCenter = Math.atan2(Math.sin(angleToCenter), Math.cos(angleToCenter));

    Rotation2d finalRotation = new Rotation2d(normalizedAngleToCenter);
    return finalRotation.rotateBy(Rotation2d.k180deg);
  }

  public void execute() {
    if (target.get() == null) {
      wrapper.resetRotationOverride();
      return;
    }
    Pose2d        pose               = currentPose.get();
    Translation2d currentTranslation = pose.getTranslation();
    Translation2d targetTranslation  = target.get();

    var targetangle         = RotateToFaceHelper.faceTowardsCenter(currentTranslation, targetTranslation);
    var currentPoseRotation = pose.getRotation();
    var rotVel              = rotationalPID.calculate(currentPoseRotation.getRadians(), targetangle.getRadians());
    wrapper.setRotationOverride(rotVel);
    currentDist = targetTranslation.getDistance(currentTranslation);

    logs.rotVel.info(rotVel);
    logs.targetRotation.info(targetangle);
    logs.currentRotation.info(currentPoseRotation);
    logs.positionErrorDeg.info(Math.toDegrees(getError()));
  }

  public void end() {
    wrapper.resetRotationOverride();
  }

  public boolean withinTolerance() {
    return Math.abs(Math.toDegrees(getError())) <= tunables.toleranceDegrees.get();
  }

  public double getError() {
    return rotationalPID.getPositionError();
  }

  public Distance getDistance() {
    return Units.Meters.of(currentDist);
  }
}
