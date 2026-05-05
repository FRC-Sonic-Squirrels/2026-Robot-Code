package frc.robot.subsystems.CTRESwerve;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib.team2930.AutoRegisterLoggers;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;

public class Telemetry {
  private final double MaxSpeed;

  /**
   * Construct a telemetry object, with the specified max speed of the robot
   *
   * @param maxSpeed Maximum speed in meters per second
   */
  public Telemetry(double maxSpeed) {
    MaxSpeed = maxSpeed;
    SignalLogger.start();

    /* Set up the module state Mechanism2d telemetry */
    for (int i = 0; i < 4; ++i) {
      SmartDashboard.putData("Module " + i, moduleMechanisms[i]);
    }
  }

  /* What to publish over networktables for telemetry */
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

  /* Robot swerve drive state */

  public final String ROOT_TABLE = "CTREDrivetrain";

  private final LoggerGroup logGroup = LoggerGroup.build(ROOT_TABLE);

  public static class Logs extends AutoRegisterLoggers {
    public LoggerEntry.Struct<Pose2d>                    drivePose;
    public LoggerEntry.Struct<ChassisSpeeds>             driveSpeeds;
    public LoggerEntry.StructArray<SwerveModuleState>    driveModuleStates;
    public LoggerEntry.StructArray<SwerveModuleState>    driveModuleTargets;
    public LoggerEntry.StructArray<SwerveModulePosition> driveModulePositions;
    public LoggerEntry.Decimal                           driveTimestamp;
    public LoggerEntry.Decimal                           driveOdometryFrequency;
  }

  private final Logs logs = logGroup.newAutoLogger(Logs.class, "Logs");

  /* Robot pose for field positioning */
  private final NetworkTable         table        = inst.getTable("Pose");
  private final DoubleArrayPublisher fieldPub     = table.getDoubleArrayTopic("robotPose").publish();
  private final StringPublisher      fieldTypePub = table.getStringTopic(".type").publish();

  /* Mechanisms to represent the swerve module states */
  private final Mechanism2d[] moduleMechanisms = new Mechanism2d[] {
      new Mechanism2d(1, 1),
      new Mechanism2d(1, 1),
      new Mechanism2d(1, 1),
      new Mechanism2d(1, 1),
  };
  /* A direction and length changing ligament for speed representation */
  private final MechanismLigament2d[] moduleSpeeds = new MechanismLigament2d[] {
      moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
      moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
      moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
      moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
  };
  /* A direction changing and length constant ligament for module direction */
  private final MechanismLigament2d[] moduleDirections = new MechanismLigament2d[] {
      moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
          .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
      moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
          .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
      moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
          .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
      moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
          .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
  };

  private final SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

  private final double[] poseArray = new double[3];

  /**
   * Accept the swerve drive state and telemeterize it to SmartDashboard and
   * SignalLogger.
   */
  public void telemeterize(SwerveDriveState state) {
    /* Telemeterize the swerve drive state */
    logs.drivePose.info(state.Pose);
    logs.driveSpeeds.info(state.Speeds);
    logs.driveModuleStates.info(state.ModuleStates);
    logs.driveModuleTargets.info(state.ModuleTargets);
    logs.driveModulePositions.info(state.ModulePositions);
    logs.driveTimestamp.info(state.Timestamp);
    logs.driveOdometryFrequency.info(1.0 / state.OdometryPeriod);

    /* Telemeterize the pose to a Field2d */
    fieldTypePub.set("Field2d");

    poseArray[0] = state.Pose.getX();
    poseArray[1] = state.Pose.getY();
    poseArray[2] = state.Pose.getRotation().getDegrees();
    fieldPub.set(poseArray);

    /* Telemeterize each module state to a Mechanism2d */
    for (int i = 0; i < 4; ++i) {
      moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
      moduleDirections[i].setAngle(state.ModuleStates[i].angle);
      moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));

      modulePositions[i] = state.ModulePositions[i];
    }
  }

  public SwerveModulePosition[] getModuleStates() {
    return modulePositions;
  }
}
