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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.*;
import frc.lib.team6328.GeomUtil;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import java.util.function.Supplier;

public class DriveToPose extends Command {
  private final String ROOT_TABLE = "DriveToPose";

  public static class Logs extends AutoRegisterLoggers {
    public LoggerEntry.Decimal        distanceMeasured;
    public LoggerEntry.Decimal        distanceSetpoint;
    public LoggerEntry.Decimal        thetaMeasured;
    public LoggerEntry.Decimal        thetaSetpoint;
    public LoggerEntry.Bool           isScheduled;
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
  private final boolean           slowMode;
  private final Supplier<Pose2d>  poseSupplier;
  private final Supplier<Pose2d>  currentRobotPose;
  private final boolean           finish;
  private final Supplier<Boolean> move;
  private final double            driveToleranceValue;
  private final Trigger           withinTolerance;

  private final ProfiledPIDController driveController = new ProfiledPIDController(0.0, 0.0, 0.0,
      new TrapezoidProfile.Constraints(0.0, 0.0));
  private final ProfiledPIDController thetaController = new ProfiledPIDController(0.0, 0.0, 0.0,
      new TrapezoidProfile.Constraints(0.0, 0.0));
  private double                      driveErrorAbs;
  private double                      thetaErrorAbs;
  private Translation2d               lastSetpointTranslation;

  /** Drives to the specified pose under full software control. */
  public DriveToPose(DrivetrainWrapper drive, boolean slowMode, Pose2d pose, Supplier<Pose2d> currentPose) {
    this(drive, slowMode, () -> pose, currentPose, true, () -> true);
  }

  /** Drives to the specified pose under full software control. */
  public DriveToPose(DrivetrainWrapper drive, boolean slowMode, Supplier<Pose2d> poseSupplier,
      Supplier<Pose2d> currentRobotPose, boolean finish, Supplier<Boolean> move) {
    this.drive               = drive;
    this.slowMode            = slowMode;
    this.poseSupplier        = poseSupplier;
    this.currentRobotPose    = currentRobotPose;
    this.finish              = finish;
    this.move                = move;
    this.driveToleranceValue = tunables.driveTolerance.get();

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

  @Override
  public void initialize() {
    drive.resetVelocityOverride();
    // Reset all controllers
    var pose        = poseSupplier.get();
    var currentPose = currentRobotPose.get();
    driveController.reset(currentPose.getTranslation().getDistance(pose.getTranslation()),
        Math.min(0.0,
            -new Translation2d(drive.getFieldRelativeVelocities().getX(), drive.getFieldRelativeVelocities().getY())
                .rotateBy(pose.getTranslation().minus(currentPose.getTranslation()).getAngle().unaryMinus()).getX()));
    thetaController.reset(currentPose.getRotation().getRadians(),
        drive.getFieldRelativeVelocities().getRotation().getRadians());
    lastSetpointTranslation = currentPose.getTranslation();

    // Update from tunable numbers
    driveController.setP(tunables.driveKP.get());
    driveController.setD(tunables.driveKD.get());
    driveController.setConstraints(new TrapezoidProfile.Constraints(
        slowMode ? tunables.driveMaxVelocitySlow.get() : tunables.driveMaxVelocity.get(),
        tunables.driveMaxAcceleration.get()));
    driveController.setTolerance(
        // slowMode ? driveToleranceSlow.get() : driveTolerance.get()
        driveToleranceValue);
    thetaController.setP(tunables.thetaKP.get());
    thetaController.setD(tunables.thetaKD.get());
    thetaController.setConstraints(new TrapezoidProfile.Constraints(
        slowMode ? tunables.thetaMaxVelocitySlow.get() : tunables.thetaMaxVelocity.get(),
        tunables.thetaMaxAcceleration.get()));
    thetaController.setTolerance(slowMode ? tunables.thetaToleranceSlow.get() : tunables.thetaTolerance.get());
  }

  @Override
  public void execute() {

    // Get current and target pose
    var currentPose = currentRobotPose.get();
    var targetPose  = poseSupplier.get();

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
    if (move.get()) {
      drive.setVelocityOverride(ChassisSpeeds.fromFieldRelativeSpeeds(driveVelocity.getX(), driveVelocity.getY(),
          thetaVelocity, currentPose.getRotation()));
    } else {
      drive.resetVelocityOverride();
    }

    // Log data
    logs.distanceMeasured.info(currentDistance);
    logs.distanceSetpoint.info(driveController.getSetpoint().position);
    logs.thetaMeasured.info(currentPose.getRotation().getRadians());
    logs.thetaSetpoint.info(thetaController.getSetpoint().position);
    logs.setpoint.info(new Pose2d(lastSetpointTranslation, new Rotation2d(thetaController.getSetpoint().position)));
    logs.goal.info(targetPose);
    logs.isScheduled.info(isScheduled());
    logs.driveControllerIsAtGoal.info(driveController.atGoal());
    logs.thetaControllerIsAtGoal.info(thetaController.atGoal());
    logs.isAtGoal.info(atGoal());
  }

  @Override
  public boolean isFinished() {
    return atGoal() && finish;
  }

  @Override
  public void end(boolean interrupted) {
    drive.resetVelocityOverride();
    logs.setpoint.info(Constants.zeroPose2d);
    logs.goal.info(Constants.zeroPose2d);
  }

  /** Checks if the robot is stopped at the final pose. */
  public boolean atGoal() {
    return
    // isScheduled() &&
    withinTolerance.getAsBoolean();
  }

  /**
   * Checks if the robot pose is within the allowed drive and theta tolerances.
   */
  public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
    return isScheduled() && Math.abs(driveErrorAbs) < driveTolerance
        && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians();
  }
}
