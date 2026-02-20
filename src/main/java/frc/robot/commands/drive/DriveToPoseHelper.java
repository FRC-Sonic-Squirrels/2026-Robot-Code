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
import frc.lib.team2930.*;
import frc.lib.team6328.GeomUtil;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;

import java.util.Objects;
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

  private final Supplier<Boolean> slowMode;
  private final double            driveToleranceValue;

  private Pose2d targetPose;

  private final ProfiledPIDController driveController = new ProfiledPIDController(0.0, 0.0, 0.0,
      new TrapezoidProfile.Constraints(0.0, 0.0));
  private final ProfiledPIDController thetaController = new ProfiledPIDController(0.0, 0.0, 0.0,
      new TrapezoidProfile.Constraints(0.0, 0.0));
  private double                      driveErrorAbs;
  private double                      thetaErrorAbs;

  /** Drives to the specified pose under full software control. */
  public DriveToPoseHelper(Supplier<Boolean> slowMode) {
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
  }

  public void resetPID(Pose2d currentPose, Pose2d fieldRelativeVelocities) {
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
    } else {
      Translation2d currentTranslation     = currentPose.getTranslation();
      Translation2d targetTranslation      = targetPose.getTranslation();
      double        currentRotationRadians = currentPose.getRotation().getRadians();
      double        targetRotationRadians  = targetPose.getRotation().getRadians();
      driveController.reset(currentTranslation.getDistance(targetTranslation), Math.min(0.0, -fieldRelativeVelocities
          .getTranslation().rotateBy(targetTranslation.minus(currentTranslation).getAngle().unaryMinus()).getX()));
      thetaController.reset(currentRotationRadians, targetRotationRadians);
    }
  }

  public ChassisSpeeds periodic(Pose2d currentPose) {

    // Calculate drive speed

    var    currentTranslation = currentPose.getTranslation();
    double currentDistance    = currentTranslation.getDistance(targetPose.getTranslation());
    double ffScaler           = MathUtil.clamp(
        (currentDistance - tunables.ffMinRadius.get()) / (tunables.ffMaxRadius.get() - tunables.ffMinRadius.get()), 0.0,
        1.0);
    driveErrorAbs = currentDistance;
    double driveVelocityScalar = driveController.getSetpoint().velocity * ffScaler
        + driveController.calculate(driveErrorAbs, 0.0);
    if (currentDistance < driveController.getPositionTolerance())
      driveVelocityScalar = 0.0;

    // Calculate theta speed
    double thetaVelocity = thetaController.getSetpoint().velocity * ffScaler
        + thetaController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    thetaErrorAbs = Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
    if (thetaErrorAbs < thetaController.getPositionTolerance())
      thetaVelocity = 0.0;

    // Command speeds
    var driveVelocity = new Pose2d(Constants.zeroTranslation2d,
        currentTranslation.minus(targetPose.getTranslation()).getAngle())
        .transformBy(GeomUtil.translationToTransform(driveVelocityScalar, 0.0)).getTranslation();

    // Log data
    logs.distanceMeasured.info(currentDistance);
    logs.distanceSetpoint.info(driveController.getSetpoint().position);
    logs.thetaMeasured.info(currentPose.getRotation().getRadians());
    logs.thetaSetpoint.info(thetaController.getSetpoint().position);
    logs.setpoint.info(new Pose2d(currentTranslation, new Rotation2d(thetaController.getSetpoint().position)));
    logs.goal.info(targetPose);
    logs.driveControllerIsAtGoal.info(driveController.atGoal());
    logs.thetaControllerIsAtGoal.info(thetaController.atGoal());
    logs.isAtGoal.info(withinTolerance(tunables.driveTolerance.get(), new Rotation2d(tunables.thetaTolerance.get())));
    return ChassisSpeeds.fromFieldRelativeSpeeds(driveVelocity.getX(), driveVelocity.getY(), thetaVelocity,
        currentPose.getRotation());
  }

  public boolean setTarget(Pose2d target) {
    boolean reset = !Objects.equals(this.targetPose, target);
    this.targetPose = target;
    return reset;
  }

  /**
   * Checks if the robot pose is within the allowed drive and theta tolerances.
   */
  private boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
    return Math.abs(driveErrorAbs) < driveTolerance && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians();
  }
}
