// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.*;
import frc.lib.team6328.GeomUtil;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import java.util.function.Supplier;

public class DriveToPoseHelper {
  private final String ROOT_TABLE = "DriveToPoseHelper";

  public static class Logs extends AutoRegisterLoggers {
    public LoggerEntry.Decimal        distanceMeasured;
    public LoggerEntry.Decimal        distanceSetpoint;
    public LoggerEntry.Decimal        thetaMeasured;
    public LoggerEntry.Decimal        thetaSetpoint;
    public LoggerEntry.Bool           driveControllerIsAtGoal;
    public LoggerEntry.Bool           thetaControllerIsAtGoal;
    public LoggerEntry.Bool           isAtGoal;
    public LoggerEntry.Struct<Pose2d> setpoint;
    public LoggerEntry.Struct<Pose2d> goal;
  }

  public final Logs logs = LoggerGroup.root.newAutoLogger(Logs.class, ROOT_TABLE);

  public static class Tunables extends AutoRegisterTunables {
    @TunableEntry(defaultValue = 3.0)
    public LoggedTunableNumber driveKP;
    @TunableEntry(defaultValue = 0.01)
    public LoggedTunableNumber driveKD;

    @TunableEntry(defaultValue = 5.0)
    public LoggedTunableNumber thetaKP;

    @TunableEntry(defaultValue = 0.0)
    public LoggedTunableNumber thetaKD;

    @TunableEntry(defaultValue = 5.0)
    public LoggedTunableNumber driveMaxVelocity;
    public LoggedTunableNumber driveMaxVelocitySlow;
    public LoggedTunableNumber driveMaxAcceleration;
    public LoggedTunableNumber thetaMaxVelocity;
    public LoggedTunableNumber thetaMaxVelocitySlow;
    public LoggedTunableNumber thetaMaxAcceleration;
    public LoggedTunableNumber driveTolerance;
    public LoggedTunableNumber driveToleranceSlow;
    public LoggedTunableNumber thetaTolerance;
    public LoggedTunableNumber thetaToleranceSlow;
    public LoggedTunableNumber ffMinRadius;
    public LoggedTunableNumber ffMaxRadius;
  }

  private final Tunables tunables = TunableNumberGroup.newAutoTunable(Tunables.class, ROOT_TABLE);

  private final DrivetrainWrapper drive;
  private final Supplier<Boolean> slowMode;
  private final double            driveToleranceValue;
  private final Trigger           withinTolerance;

  private Pose2d targetPose;

  private final ProfiledPIDController driveController = new ProfiledPIDController(0.0, 0.0, 0.0,
      new TrapezoidProfile.Constraints(0.0, 0.0));
  private final ProfiledPIDController thetaController = new ProfiledPIDController(0.0, 0.0, 0.0,
      new TrapezoidProfile.Constraints(0.0, 0.0));
  private double                      driveErrorAbs;
  private double                      thetaErrorAbs;
  private Translation2d               lastSetpointTranslation;

  /** Drives to the specified pose under full software control. */
  public DriveToPoseHelper(DrivetrainWrapper drive, Supplier<Boolean> slowMode) {
    this.drive               = drive;
    this.slowMode            = slowMode;
    this.driveToleranceValue = 0.1;

    tunables.driveMaxVelocity.initDefault(5);
    tunables.driveMaxVelocitySlow.initDefault(Units.Inches.of(50.0).in(Units.Meters));
    tunables.driveMaxAcceleration.initDefault(Units.Inches.of(118.0).in(Units.Meters));
    tunables.thetaMaxVelocity.initDefault(Math.toRadians(360.0));
    tunables.thetaMaxVelocitySlow.initDefault(Math.toRadians(90.0));
    tunables.thetaMaxAcceleration.initDefault(Math.toRadians(720.0));
    tunables.driveTolerance.initDefault(0.02);
    tunables.driveToleranceSlow.initDefault(0.05);
    tunables.thetaTolerance.initDefault(0.01);
    tunables.thetaToleranceSlow.initDefault(Math.toRadians(3.0));
    tunables.ffMinRadius.initDefault(0.0);
    tunables.ffMaxRadius.initDefault(1);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    withinTolerance = new Trigger(() -> driveController.atGoal() && thetaController.atGoal()).debounce(0.3);
  }

  public void initialize(Pose2d currentPose) {
    drive.resetVelocityOverride();
    // Reset all controllers

    // Update from tunable numbers
    driveController.setP(tunables.driveKP.get());
    driveController.setD(tunables.driveKD.get());
    driveController.setConstraints(new TrapezoidProfile.Constraints(
        slowMode.get() ? tunables.driveMaxVelocitySlow.get() : tunables.driveMaxVelocity.get(),
        tunables.driveMaxAcceleration.get()));
    driveController.setTolerance(
        // slowMode ? driveToleranceSlow.get() : driveTolerance.get()
        driveToleranceValue);
    thetaController.setP(tunables.thetaKP.get());
    thetaController.setD(tunables.thetaKD.get());
    thetaController.setConstraints(new TrapezoidProfile.Constraints(
        slowMode.get() ? tunables.thetaMaxVelocitySlow.get() : tunables.thetaMaxVelocity.get(),
        tunables.thetaMaxAcceleration.get()));
    thetaController.setTolerance(slowMode.get() ? tunables.thetaToleranceSlow.get() : tunables.thetaTolerance.get());
    if (targetPose == null) {
      driveController.reset(0);
      thetaController.reset(0);
      lastSetpointTranslation = currentPose.getTranslation();
    } else {
      driveController.reset(currentPose.getTranslation().getDistance(targetPose.getTranslation()),
          Math.min(0.0,
              -new Translation2d(drive.getFieldRelativeVelocities().getX(), drive.getFieldRelativeVelocities().getY())
                  .rotateBy(targetPose.getTranslation().minus(currentPose.getTranslation()).getAngle().unaryMinus())
                  .getX()));
      thetaController.reset(currentPose.getRotation().getRadians(),
          drive.getFieldRelativeVelocities().getRotation().getRadians());
      lastSetpointTranslation = currentPose.getTranslation();
    }
  }

  public void periodic(Pose2d currentPose) {

    // Calculate drive speed
    double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    double ffScaler        = MathUtil.clamp(
        (currentDistance - tunables.ffMinRadius.get()) / (tunables.ffMaxRadius.get() - tunables.ffMinRadius.get()), 0.0,
        1.0);
    driveErrorAbs = currentDistance;
    driveController.reset(lastSetpointTranslation.getDistance(targetPose.getTranslation()),
        driveController.getSetpoint().velocity);
    double driveVelocityScalar = driveController.getSetpoint().velocity * ffScaler
        + driveController.calculate(driveErrorAbs, 0.0);
    if (currentDistance < driveController.getPositionTolerance())
      driveVelocityScalar = 0.0;
    lastSetpointTranslation = new Pose2d(targetPose.getTranslation(),
        currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
        .transformBy(GeomUtil.translationToTransform(driveController.getSetpoint().position, 0.0)).getTranslation();

    // Calculate theta speed
    double thetaVelocity = thetaController.getSetpoint().velocity * ffScaler
        + thetaController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    thetaErrorAbs = Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
    if (thetaErrorAbs < thetaController.getPositionTolerance())
      thetaVelocity = 0.0;

    // Command speeds
    var driveVelocity = new Pose2d(Constants.zeroTranslation2d,
        currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
        .transformBy(GeomUtil.translationToTransform(driveVelocityScalar, 0.0)).getTranslation();
    drive.setVelocityOverride(ChassisSpeeds.fromFieldRelativeSpeeds(driveVelocity.getX(), driveVelocity.getY(),
        thetaVelocity, currentPose.getRotation()));

    // Log data
    logs.distanceMeasured.info(currentDistance);
    logs.distanceSetpoint.info(driveController.getSetpoint().position);
    logs.thetaMeasured.info(currentPose.getRotation().getRadians());
    logs.thetaSetpoint.info(thetaController.getSetpoint().position);
    logs.setpoint.info(new Pose2d(lastSetpointTranslation, new Rotation2d(thetaController.getSetpoint().position)));
    logs.goal.info(targetPose);
    logs.driveControllerIsAtGoal.info(driveController.atGoal());
    logs.thetaControllerIsAtGoal.info(thetaController.atGoal());
    logs.isAtGoal.info(atGoal());
  }

  public void setTarget(Pose2d target, Pose2d currentPose) {
    if (this.targetPose != target) {
      initialize(currentPose);
    }
    this.targetPose = target;
  }

  public void end() {
    drive.resetVelocityOverride();
    logs.setpoint.info(Constants.zeroPose2d);
    logs.goal.info(Constants.zeroPose2d);
  }

  /** Checks if the robot is stopped at the final pose. */
  public boolean atGoal() {
    return withinTolerance.getAsBoolean();
  }

  /**
   * Checks if the robot pose is within the allowed drive and theta tolerances.
   */
  public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
    return Math.abs(driveErrorAbs) < driveTolerance && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians();
  }
}
