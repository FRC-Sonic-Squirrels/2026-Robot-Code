package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.team2930.*;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;

public class DrivetrainWrapper {
  private final String ROOT_TABLE = "DrivetrainWrapper";

  public static class Logs extends AutoRegisterLoggers {
    public LoggerEntry.Struct<ChassisSpeeds> chassisSpeedsBase;
    public LoggerEntry.Struct<ChassisSpeeds> chassisSpeedsOverride;
    public LoggerEntry.Decimal               omegaOverride;
    public LoggerEntry.Decimal               gyroDrift;
  }

  public final Logs logs = LoggerGroup.root.newAutoLogger(Logs.class, ROOT_TABLE);

  public static class Tunables extends AutoRegisterTunables {
    @TunableEntry(defaultValue = 100.0)
    public LoggedTunableNumber driveSpeedPercent;

    @TunableEntry(defaultValue = 100.0)
    public LoggedTunableNumber rotateSpeedPercent;
  }

  private final Tunables tunables = TunableNumberGroup.newAutoTunable(Tunables.class, ROOT_TABLE);

  private final Drivetrain drivetrain;

  private ChassisSpeeds chassisSpeedsBase = new ChassisSpeeds();
  private ChassisSpeeds chassisSpeedsOverride;
  private double        omegaOverride     = Double.NaN;

  public DrivetrainWrapper(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  /** Updated periodically, updates swerve state. */
  public void apply() {
    // Log chassis speeds
    if (chassisSpeedsBase != null) {
      logs.chassisSpeedsBase.info(chassisSpeedsBase);
    }
    if (chassisSpeedsOverride != null) {
      logs.chassisSpeedsOverride.info(chassisSpeedsOverride);
    }
    logs.omegaOverride.info(omegaOverride);

    ChassisSpeeds chassisSpeeds = chassisSpeedsBase;

    boolean prioritizeRotation;

    // If omega override, use that for omega
    if (Double.isFinite(omegaOverride)) {
      prioritizeRotation = true;
      chassisSpeeds      = new ChassisSpeeds(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond,
          omegaOverride);
    } else {
      prioritizeRotation = false;
    }

    // If chassisSpeedsOverride, replace chassisSpeeds with that

    if (chassisSpeedsOverride != null) {
      chassisSpeeds = chassisSpeedsOverride;
    }

    // Hack to make it run forward!!!!!!!!!
    chassisSpeeds.vxMetersPerSecond     *= tunables.driveSpeedPercent.get() / 100.0;
    chassisSpeeds.vyMetersPerSecond     *= tunables.driveSpeedPercent.get() / 100.0;
    chassisSpeeds.omegaRadiansPerSecond *= tunables.rotateSpeedPercent.get() / 100.0;

    // Run drivetrain

    drivetrain.runVelocity(chassisSpeeds, prioritizeRotation, chassisSpeedsOverride != null);

    var pose1 = getHubPoseEstimatorPose(false);
    var pose2 = getHubPoseEstimatorPose(true);

    // Diff between strictly gyro estimate and vision assisted estimate
    logs.gyroDrift.info(pose1.getRotation().minus(pose2.getRotation()));
  }

  // Setters

  /**
   * Sets robot-centric chassis speeds
   *
   * @param chassisSpeeds speeds (m/s, rad/s)
   */
  public void setVelocity(ChassisSpeeds chassisSpeeds) {
    if (chassisSpeeds == null)
      chassisSpeeds = new ChassisSpeeds();
    chassisSpeedsBase = chassisSpeeds;
  }

  /**
   * Sets robot-centric chassis speeds
   *
   * <p>
   * Overrides 'setVelocity' and 'setRotationOverride'
   *
   * @param chassisSpeeds speeds (m/s, rad/s)
   */
  public void setVelocityOverride(ChassisSpeeds chassisSpeeds) {
    chassisSpeedsOverride = chassisSpeeds;
  }

  /** Gives velocity control back to base */
  public void resetVelocityOverride() {
    chassisSpeedsOverride = null;
  }

  /**
   * Sets robot rotational vel
   *
   * <p>
   * Overrides 'setVelocity'
   *
   * @param omega
   */
  public void setRotationOverride(double omega) {
    omegaOverride = omega;
  }

  /** Gives rotational vel control back to base */
  public void resetRotationOverride() {
    omegaOverride = Double.NaN;
  }

  public SysIdRoutine.Mechanism getSysIdMechanism() {
    // No log consumer, since data is recorded by AdvantageKit
    return new SysIdRoutine.Mechanism((voltage) -> drivetrain.runCharacterizationVolts(voltage.in(Units.Volts)), null,
        drivetrain);
  }

  public void setPose(Pose2d pose) {
    drivetrain.setPose(pose);
  }

  // Getters

  public Subsystem getRequirements() {
    return drivetrain;
  }

  public Pose2d getHubPoseEstimatorPose(boolean prioritizeGyro) {
    var pose = drivetrain.getHubPoseEstimatorPose();
    if (Constants.unusedCode && prioritizeGyro) {
      pose = new Pose2d(pose.getTranslation(), getRotationGyroOnly());
    }

    return pose;
  }

  public double getVisionStaleness() {
    return drivetrain.getHubVisionStaleness();
  }

  public LinearVelocity getMaxLinearSpeed() {
    return Units.MetersPerSecond.of(drivetrain.getMaxLinearSpeedMetersPerSec());
  }

  public AngularVelocity getMaxAngularSpeed() {
    return Units.RadiansPerSecond.of(drivetrain.getMaxAngularSpeedRadPerSec());
  }

  public Pose2d getFieldRelativeVelocities() {
    return drivetrain.getFieldRelativeVelocities();
  }

  public Pose2d getRobotCentricVelocities() {
    return drivetrain.getRobotCentricVelocities();
  }

  public Rotation2d getRotationGyroOnly() {
    return drivetrain.getRotationGyroOnly();
  }

  public Angle[] getModuleRotations() {
    return drivetrain.getModuleDriveRotations();
  }

  public ChassisSpeeds getCurrentRobotRelativeChassisSpeeds() {
    return drivetrain.getChassisSpeeds();
  }

  public Pose2d getRawOdometryPose() {
    return drivetrain.getRawOdometryPose();
  }

  public double getLinearVel() {
    return drivetrain.getLinearVel();
  }
}
