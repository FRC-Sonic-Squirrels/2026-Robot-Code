package frc.robot.configs;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.config.ModuleConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.lib.constants.SwerveModuleConstants;
import frc.lib.team2930.AutoRegisterTunables;
import frc.lib.team2930.TunableEntry;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import frc.robot.subsystems.swerve.SwerveModuleIOTalonFX;
import frc.robot.subsystems.swerve.SwerveModules;
import frc.robot.subsystems.vision.VisionModuleConfiguration;

public class RobotConfig2026 extends RobotConfig {
  public static class Tunables extends AutoRegisterTunables {
    // ---------- SWERVE STEERING MOTOR PID CONSTANTS -----------
    // FIXE: RN copied from Mechanical advantage (6328) 2023 codebase. Should learn
    // to tune
    // ourselves
    @TunableEntry(defaultValue = 200.0)
    public LoggedTunableNumber ANGLE_KP;

    @TunableEntry(defaultValue = 2.0)
    public LoggedTunableNumber ANGLE_KD;

    // ---------- SWERVE DRIVE MOTOR PID + KS + KV + KA CONSTANTS -------------
    @TunableEntry(defaultValue = 1.0)
    public LoggedTunableNumber DRIVE_KP;

    @TunableEntry(defaultValue = 0.0)
    public LoggedTunableNumber DRIVE_KD;

    @TunableEntry(defaultValue = 0.0)
    public LoggedTunableNumber DRIVE_KS;

    @TunableEntry(defaultValue = 0.0)
    public LoggedTunableNumber DRIVE_KV;

    @TunableEntry(defaultValue = 0.0)
    public LoggedTunableNumber DRIVE_KA;

    // ------- AUTONOMOUS CONSTANTS -------
    @TunableEntry(defaultValue = 5.0)
    public LoggedTunableNumber AUTO_MAX_SPEED_METERS_PER_SECOND;

    @TunableEntry(defaultValue = 5.0)
    public LoggedTunableNumber AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED;

    @TunableEntry(defaultValue = Math.PI * 2)
    public LoggedTunableNumber AUTO_MAX_ANGULAR_VEL_RADIANS_PER_SECOND;

    @TunableEntry(defaultValue = Math.PI * 4)
    public LoggedTunableNumber AUTO_MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED;

    @TunableEntry(defaultValue = 2.4)
    public LoggedTunableNumber AUTO_TRANSLATION_KP;

    @TunableEntry(defaultValue = 0.0)
    public LoggedTunableNumber AUTO_TRANSLATION_KI;

    @TunableEntry(defaultValue = 0.0)
    public LoggedTunableNumber AUTO_TRANSLATION_KD;

    @TunableEntry(defaultValue = 4.0)
    public LoggedTunableNumber AUTO_THETA_KP;

    @TunableEntry(defaultValue = 0.0)
    public LoggedTunableNumber AUTO_THETA_KI;

    @TunableEntry(defaultValue = 0.0)
    public LoggedTunableNumber AUTO_THETA_KD;
  }

  private final Tunables tunables = TunableNumberGroup.newAutoTunable(Tunables.class, ROOT_TABLE());

  // ------------ SWERVE ---------------------
  private final Distance WHEEL_RADIUS = Units.Inches.of(1.929);

  // ------ SWERVE MODULE CONFIGURATIONS: CANID + OFFSET + INVERTS --------------
  //
  // Calibration procedure:
  //
  // 1: Set all rotation angles to zero => Deploy.
  //
  // 2: Manually turn each wheel clockwise.
  // Make sure the value of "TurnAbsolutionPosition" increases.
  // If not, configure the CAN encoder to be inverted.
  //
  // 3: Measure all the absolute positions of the CAN encoders.
  // Record the values in the Rotation2D section => Deploy
  //
  // 4: Check the rotation of the drive, steer.
  // Drive forward. If a wheel turns the opposite way, invert its drive motor.
  // Turn left. If a wheel rotates the opposite way, invert its steer motor.
  //
  private final IndividualSwerveModuleConfig FRONT_LEFT_MODULE_CONFIG = new IndividualSwerveModuleConfig(1, 11, 21,
      Rotation2d.fromRotations(-69.608 / 360), InvertedValue.CounterClockwise_Positive,
      InvertedValue.CounterClockwise_Positive, InvertedValue.Clockwise_Positive);
  // 1
  private final IndividualSwerveModuleConfig FRONT_RIGHT_MODULE_CONFIG = new IndividualSwerveModuleConfig(2, 12, 22,
      Rotation2d.fromRotations(-22.5 / 360), InvertedValue.Clockwise_Positive, InvertedValue.CounterClockwise_Positive,
      InvertedValue.Clockwise_Positive);
  // 2
  private final IndividualSwerveModuleConfig BACK_LEFT_MODULE_CONFIG = new IndividualSwerveModuleConfig(3, 13, 23,
      Rotation2d.fromRotations(-132.803 / 360), InvertedValue.CounterClockwise_Positive,
      InvertedValue.CounterClockwise_Positive, InvertedValue.Clockwise_Positive);
  // 3
  private final IndividualSwerveModuleConfig BACK_RIGHT_MODULE_CONFIG = new IndividualSwerveModuleConfig(4, 14, 24,
      Rotation2d.fromRotations(0.615 / 360), InvertedValue.Clockwise_Positive, InvertedValue.CounterClockwise_Positive,
      InvertedValue.CounterClockwise_Positive);

  // -------- SWERVE CURRENT LIMITS ---------
  private final CurrentLimitsConfigs DRIVE_TALON_CURRENT_LIMIT_CONFIGS = new CurrentLimitsConfigs()
      .withSupplyCurrentLimit(20).withSupplyCurrentLimitEnable(true);

  private final CurrentLimitsConfigs STEER_TALON_CURRENT_LIMIT_CONFIGS = new CurrentLimitsConfigs()
      .withSupplyCurrentLimit(25).withSupplyCurrentLimitEnable(true);

  // --------- SWERVE GEAR RATIO ---------
  public final double SWERVE_DRIVE_GEAR_RATIO = SwerveModuleConstants.MK5n.LEVEL_2_GEARING_DRIVE_GEAR_RATIO;
  public final double SWERVE_STEER_GEAR_RATIO = SwerveModuleConstants.MK5n.GEARING_TURN_GEAR_RATIO;

  // -------- GYRO CAN ID ---------
  private final int GYRO_CAN_ID = 5;

  // -------- GYRO OFFSETS --------

  private final double GYRO_MOUNTING_PITCH = 0.0;

  private final double GYRO_MOUNTING_ROLL = -180.0;

  private final double GYRO_MOUNTING_YAW = 90.0;

  // -------- CAN BUS NAME -----------
  private final CANBus CAN_BUS_NAME = new CANBus("CANivore");

  // -------- ROBOT DIMENSIONS -----------
  // front to back
  private final Distance TRACK_WIDTH_X = Units.Inches.of(21.75);
  // left to right
  private final Distance TRACK_WIDTH_Y = Units.Inches.of(21.75);

  // ------- ROBOT MAX SPEED --------
  private final double MAX_VELOCITY_METERS_PER_SECOND       = 5.0;
  private final double MAX_COAST_VELOCITY_METERS_PER_SECOND = 0.05;

  private final com.pathplanner.lib.config.RobotConfig PATH_PLANNER_CONFIG = new com.pathplanner.lib.config.RobotConfig(
      Units.Kilogram.of(1), Units.KilogramSquareMeters.of(1),
      new ModuleConfig(WHEEL_RADIUS, Units.MetersPerSecond.of(MAX_VELOCITY_METERS_PER_SECOND), 0.65,
          DCMotor.getKrakenX60Foc(4), SWERVE_DRIVE_GEAR_RATIO,
          Units.Amps.of(DRIVE_TALON_CURRENT_LIMIT_CONFIGS.SupplyCurrentLimit), 1),
      getModuleTranslations());

  // ---- VISION -------
  public final Transform3d SHOOTER_SIDE_LEFT = new Transform3d(
      new Translation3d(Units.Inches.of(-12.5).in(Units.Meters), Units.Inches.of(-5.1405).in(Units.Meters),
          Units.Inches.of(10).in(Units.Meters)),
      new Rotation3d(Math.toRadians(0.0), Math.toRadians(-25.0), Math.toRadians(-20.0 + 180 + .7)));

  public final Transform3d SHOOTER_SIDE_RIGHT = new Transform3d(
      new Translation3d(Units.Inches.of(-12.5).in(Units.Meters), Units.Inches.of(5.1405).in(Units.Meters),
          Units.Inches.of(10).in(Units.Meters)),
      new Rotation3d(Math.toRadians(0.0), Math.toRadians(-25.0), Math.toRadians(20.0 + 180)));

  public final String SHOOTER_SIDE_LEFT_CAMERA_NAME  = "1_Shooter_Left_See3Cam";
  public final String SHOOTER_SIDE_RIGHT_CAMERA_NAME = "2_Shooter_Right_See3Cam";

  public final AprilTagFields APRIL_TAG_FIELD = AprilTagFields.k2026RebuiltWelded;

  /*
   *
   *
   * FUNCTIONS RETURNING CONSTANTS:
   *
   * YOU MUST CHANGED 2 METHODS: getSwerveModuleObjects() getVisionModuleObjects()
   *
   */

  @Override
  public SwerveModules getSwerveModuleObjects() {
    return new SwerveModules(new SwerveModule(0, this, new SwerveModuleIOTalonFX(this, FRONT_LEFT_MODULE_CONFIG)),
        new SwerveModule(1, this, new SwerveModuleIOTalonFX(this, FRONT_RIGHT_MODULE_CONFIG)),
        new SwerveModule(2, this, new SwerveModuleIOTalonFX(this, BACK_LEFT_MODULE_CONFIG)),
        new SwerveModule(3, this, new SwerveModuleIOTalonFX(this, BACK_RIGHT_MODULE_CONFIG)));
  }

  @Override
  public SwerveModules getReplaySwerveModuleObjects() {
    return new SwerveModules(new SwerveModule(0, this, new SwerveModuleIO.Fake()),
        new SwerveModule(1, this, new SwerveModuleIO.Fake()), new SwerveModule(2, this, new SwerveModuleIO.Fake()),
        new SwerveModule(3, this, new SwerveModuleIO.Fake()));
  }

  @Override
  public VisionModuleConfiguration[] getVisionModuleObjects() {
    return new VisionModuleConfiguration[] {
        VisionModuleConfiguration.build(SHOOTER_SIDE_LEFT_CAMERA_NAME, SHOOTER_SIDE_LEFT),
        VisionModuleConfiguration.build(SHOOTER_SIDE_RIGHT_CAMERA_NAME, SHOOTER_SIDE_RIGHT) };
  }

  @Override
  public VisionModuleConfiguration[] getReplayVisionModules() {
    return new VisionModuleConfiguration[] {
        VisionModuleConfiguration.buildReplayStub(SHOOTER_SIDE_LEFT_CAMERA_NAME, SHOOTER_SIDE_LEFT),
        VisionModuleConfiguration.buildReplayStub(SHOOTER_SIDE_RIGHT_CAMERA_NAME, SHOOTER_SIDE_RIGHT) };
  }

  @Override
  public AprilTagFieldLayout getAprilTagFieldLayout() {
    return AprilTagFieldLayout.loadField(APRIL_TAG_FIELD);
  }

  @Override
  public boolean getPhoenix6Licensed() {
    return true;
  }

  @Override
  public IndividualSwerveModuleConfig[] getIndividualModuleConfigurations() {
    return new IndividualSwerveModuleConfig[] { FRONT_LEFT_MODULE_CONFIG, FRONT_RIGHT_MODULE_CONFIG,
        BACK_LEFT_MODULE_CONFIG, BACK_RIGHT_MODULE_CONFIG };
  }

  @Override
  public int getGyroCANID() {
    return GYRO_CAN_ID;
  }

  @Override
  public Distance getTrackWidth_Y() {
    return TRACK_WIDTH_Y;
  }

  @Override
  public Distance getTrackWidth_X() {
    return TRACK_WIDTH_X;
  }

  @Override
  public double getRobotMaxLinearVelocity() {
    return MAX_VELOCITY_METERS_PER_SECOND;
  }

  @Override
  public double getRobotMaxCoastVelocity() {
    return MAX_COAST_VELOCITY_METERS_PER_SECOND;
  }

  @Override
  public CANBus getCANBus() {
    return CAN_BUS_NAME;
  }

  @Override
  public LoggedTunableNumber getDriveKS() {
    return tunables.DRIVE_KS;
  }

  @Override
  public LoggedTunableNumber getDriveKV() {
    return tunables.DRIVE_KV;
  }

  @Override
  public LoggedTunableNumber getDriveKA() {
    return tunables.DRIVE_KA;
  }

  @Override
  public LoggedTunableNumber getDriveKP() {
    return tunables.DRIVE_KP;
  }

  @Override
  public LoggedTunableNumber getDriveKD() {
    return tunables.DRIVE_KD;
  }

  @Override
  public LoggedTunableNumber getAngleKP() {
    return tunables.ANGLE_KP;
  }

  @Override
  public LoggedTunableNumber getAngleKD() {
    return tunables.ANGLE_KD;
  }

  @Override
  public LoggedTunableNumber getPathingMaxSpeedMPS() {
    return tunables.AUTO_MAX_SPEED_METERS_PER_SECOND;
  }

  @Override
  public LoggedTunableNumber getPathingMaxAccelerationMPSPS() {
    return tunables.AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED;
  }

  @Override
  public LoggedTunableNumber getPathingMaxAngularVelocityRadPerSecond() {
    return tunables.AUTO_MAX_ANGULAR_VEL_RADIANS_PER_SECOND;
  }

  @Override
  public LoggedTunableNumber getPathingMaxAngularAccelerationRadPerSecondSquared() {
    return tunables.AUTO_MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED;
  }

  @Override
  public PIDController getAutoTranslationPidController() {
    return new PIDController(tunables.AUTO_TRANSLATION_KP.get(), tunables.AUTO_TRANSLATION_KI.get(),
        tunables.AUTO_TRANSLATION_KD.get());
  }

  @Override
  public PIDController getAutoThetaPidController() {
    return new PIDController(tunables.AUTO_THETA_KP.get(), tunables.AUTO_THETA_KI.get(), tunables.AUTO_THETA_KD.get());
  }

  @Override
  public Distance getWheelRadius() {
    return WHEEL_RADIUS;
  }

  @Override
  public double getSwerveModuleDriveGearRatio() {
    return SWERVE_DRIVE_GEAR_RATIO;
  }

  @Override
  public double getSwerveModuleTurnGearRatio() {
    return SWERVE_STEER_GEAR_RATIO;
  }

  @Override
  public CurrentLimitsConfigs getDriveTalonCurrentLimitConfig() {
    return DRIVE_TALON_CURRENT_LIMIT_CONFIGS;
  }

  @Override
  public CurrentLimitsConfigs getSteerTalonCurrentLimitConfig() {
    return STEER_TALON_CURRENT_LIMIT_CONFIGS;
  }

  @Override
  public double getGyroMountingPitch() {
    return GYRO_MOUNTING_PITCH;
  }

  @Override
  public double getGyroMountingRoll() {
    return GYRO_MOUNTING_ROLL;
  }

  @Override
  public double getGyroMountingYaw() {
    return GYRO_MOUNTING_YAW;
  }

  @Override
  public com.pathplanner.lib.config.RobotConfig pathPlannerConfig() {
    return PATH_PLANNER_CONFIG;
  }
}
