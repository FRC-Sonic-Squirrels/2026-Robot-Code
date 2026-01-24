// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import java.util.function.Supplier;

public class RotateToFaceHelper {
  private static final String ROOT_TABLE = "RotateToFaceHelper";

  private static final LoggerGroup logGroup = LoggerGroup.build(ROOT_TABLE);
  private static final LoggerEntry.Decimal log_rotVel = logGroup.buildDecimal("rotVel");
  private static final LoggerEntry.Decimal log_targetRotation =
      logGroup.buildDecimal("targetRotation");
  private static final LoggerEntry.Decimal log_currentRotation =
      logGroup.buildDecimal("currentRotation");
  private static final LoggerEntry.Decimal log_positionErrorDeg =
      logGroup.buildDecimal("positionErrorDeg");

  private static final TunableNumberGroup groupTunable = new TunableNumberGroup(ROOT_TABLE);
  private static final LoggedTunableNumber tunableKP = groupTunable.build("kP", 4.9);
  private static final LoggedTunableNumber cruiseVelocity =
      groupTunable.build("cruiseVelocity", 2 * Math.PI);
  private static final LoggedTunableNumber maxAcceleration =
      groupTunable.build("maxAcceleration", 4 * Math.PI);
  private static final LoggedTunableNumber toleranceDegrees =
      groupTunable.build("toleranceDegrees", 1);

  private final DrivetrainWrapper wrapper;
  private final Supplier<Translation2d> target;
  private final Supplier<Pose2d> currentPose;
  private final ProfiledPIDController rotationalPID =
      new ProfiledPIDController(0, 0, 0, new Constraints(0.0, 0.0));

  /** Creates a new RotateToFaceHelper . */
  public RotateToFaceHelper(
      DrivetrainWrapper wrapper, Supplier<Translation2d> target, Supplier<Pose2d> pose2d) {
    this.wrapper = wrapper;
    this.target = target;
    this.currentPose = pose2d;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public void initialize() {
    rotationalPID.reset(
        currentPose.get().getRotation().getRadians(),
        wrapper.getFieldRelativeVelocities().getRotation().getRadians());
    rotationalPID.setP(tunableKP.get());
    rotationalPID.setConstraints(new Constraints(cruiseVelocity.get(), maxAcceleration.get()));
    rotationalPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  private Rotation2d faceTowardsCenter(Translation2d target) {

    double targetX = target.getX();
    double targetY = target.getY();

    double robotX = currentPose.get().getTranslation().getX();
    double robotY = currentPose.get().getTranslation().getY();

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
    var targetangle = faceTowardsCenter(target.get());
    var currentPoseRotation = currentPose.get().getRotation();
    var rotVel =
        rotationalPID.calculate(currentPoseRotation.getRadians(), targetangle.getRadians());
    wrapper.setRotationOverride(rotVel);

    log_rotVel.info(rotVel);
    log_targetRotation.info(targetangle);
    log_currentRotation.info(currentPoseRotation);
    log_positionErrorDeg.info(Math.toDegrees(rotationalPID.getPositionError()));
  }

  public void end() {
    wrapper.resetRotationOverride();
  }

  public boolean withinTolerance() {
    return Math.abs(Math.toDegrees(rotationalPID.getPositionError())) <= toleranceDegrees.get();
  }

  public double getError() {
    return rotationalPID.getPositionError();
  }
}
