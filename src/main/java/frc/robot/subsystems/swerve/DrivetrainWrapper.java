package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

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
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DrivetrainWrapper {
  private final String ROOT_TABLE = "DrivetrainWrapper";

  public static class Logs extends AutoRegisterLoggers {
    public LoggerEntry.Struct<ChassisSpeeds> chassisSpeeds;
    public LoggerEntry.Struct<ChassisSpeeds> chassisSpeedsBase;
    public LoggerEntry.Struct<ChassisSpeeds> chassisSpeedsOverride;
    public LoggerEntry.Decimal               omegaOverride;
    public LoggerEntry.Decimal               gyroDrift;
  }

  public final Logs logs = LoggerGroup.root.newAutoLogger(Logs.class, ROOT_TABLE);

  public static class Tunables extends AutoRegisterTunables {
    @TunableEntry(defaultValue = 6)
    public LoggedTunableNumber maxDriveSpeedMetersPerSecond;

    @TunableEntry(defaultValue = 720.0)
    public LoggedTunableNumber maxRotateSpeedPercentDegreesPerSecond;

    @TunableEntry(defaultValue = 1.0)
    public LoggedTunableNumber useCTREDrive;
  }

  private final Tunables tunables = TunableNumberGroup.newAutoTunable(Tunables.class, ROOT_TABLE);

  private final Drivetrain              drivetrain;
  private final CommandSwerveDrivetrain CTREDrivetrain;

  private ChassisSpeeds chassisSpeedsBase = new ChassisSpeeds();
  private ChassisSpeeds chassisSpeedsOverride;
  private double        omegaOverride     = Double.NaN;

  private final SwerveRequest.FieldCentric drive;

  public DrivetrainWrapper(Drivetrain drivetrain, CommandSwerveDrivetrain CTREDrivetrain) {
    this.drivetrain     = drivetrain;
    this.CTREDrivetrain = CTREDrivetrain;
    drive               = new SwerveRequest.FieldCentric().withDeadband(0.0).withRotationalDeadband(0.0)
        .withDriveRequestType(DriveRequestType.Velocity);
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

    var    velocity    = Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    double maxVelocity = tunables.maxDriveSpeedMetersPerSecond.get();
    if (velocity > maxVelocity) {
      chassisSpeeds.vxMetersPerSecond *= maxVelocity / velocity;
      chassisSpeeds.vyMetersPerSecond *= maxVelocity / velocity;
    }

    double maxRotation = Units.Degree.of(tunables.maxRotateSpeedPercentDegreesPerSecond.get()).in(Units.Radians);
    double absRotation = Math.abs(chassisSpeeds.omegaRadiansPerSecond);
    if (absRotation > maxRotation) {
      chassisSpeeds.omegaRadiansPerSecond *= maxRotation / absRotation;
    }

    // Run drivetrain

    logs.chassisSpeeds.info(chassisSpeeds);

    if (!useCTREDrive()) {
      drivetrain.runVelocity(chassisSpeeds, prioritizeRotation, chassisSpeedsOverride != null);
    } else {
      CTREDrivetrain.setControl(drive.withVelocityX(chassisSpeeds.vxMetersPerSecond)
          .withVelocityY(chassisSpeeds.vyMetersPerSecond).withRotationalRate(chassisSpeeds.omegaRadiansPerSecond));
    }

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
    if (chassisSpeeds == null) {
      chassisSpeeds = new ChassisSpeeds();
    }
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
    if (useCTREDrive()) {
      // FIXME: This does it's own sysId Stuff
      return null;// CTREDrivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward);
    }
    // No log consumer, since data is recorded by AdvantageKit
    return new SysIdRoutine.Mechanism((voltage) -> drivetrain.runCharacterizationVolts(voltage.in(Units.Volts)), null,
        drivetrain);
  }

  public void setPose(Pose2d pose) {
    if (useCTREDrive()) {
      CTREDrivetrain.resetPose(pose);
    }
    drivetrain.setPose(pose);
  }

  // Getters

  public Subsystem[] getRequirements() {
    return new Subsystem[] {
        drivetrain,
        CTREDrivetrain
    };
  }

  public Pose2d getHubPoseEstimatorPose(boolean prioritizeGyro) {
    var pose = drivetrain.getHubPoseEstimatorPose();
    if (Constants.unusedCode && prioritizeGyro) {
      pose = new Pose2d(pose.getTranslation(), getRotationGyroOnly());
    }

    return pose;
  }

  public double getVisionStaleness() {
    if (useCTREDrive()) {
      return CTREDrivetrain.getVisionStaleness();
    }
    return drivetrain.getHubVisionStaleness();
  }

  public LinearVelocity getMaxLinearSpeed() {
    return Units.MetersPerSecond.of(drivetrain.getMaxLinearSpeedMetersPerSec());
  }

  public AngularVelocity getMaxAngularSpeed() {
    return Units.RadiansPerSecond.of(drivetrain.getMaxAngularSpeedRadPerSec());
  }

  public Pose2d getFieldRelativeVelocities() {
    if (useCTREDrive()) {
      return CTREDrivetrain.getFieldRelativeVelocities();
    }
    return drivetrain.getFieldRelativeVelocities();
  }

  public Pose2d getRobotCentricVelocities() {
    if (useCTREDrive()) {
      return CTREDrivetrain.getFieldRelativeVelocities();
    }
    return drivetrain.getRobotCentricVelocities();
  }

  public Rotation2d getRotationGyroOnly() {
    if (useCTREDrive()) {
      return CTREDrivetrain.getRotationGyroOnly();
    }
    return drivetrain.getRotationGyroOnly();
  }

  public Angle[] getModuleRotations() {
    if (useCTREDrive()) {
      return CTREDrivetrain.getModuleDriveRotations();
    }
    return drivetrain.getModuleDriveRotations();
  }

  public ChassisSpeeds getCurrentRobotRelativeChassisSpeeds() {
    if (useCTREDrive()) {
      return CTREDrivetrain.getChassisSpeeds();
    }
    return drivetrain.getChassisSpeeds();
  }

  public Pose2d getRawOdometryPose() {
    if (useCTREDrive()) {
      return CTREDrivetrain.getRawOdometryPose();
    }
    return drivetrain.getRawOdometryPose();
  }

  public double getLinearVel() {
    if (useCTREDrive()) {
      return CTREDrivetrain.getState().Speeds.vxMetersPerSecond;
    }
    return drivetrain.getLinearVel();
  }

  public boolean useCTREDrive() {
    return tunables.useCTREDrive.get() != 0.0;
  }
}
