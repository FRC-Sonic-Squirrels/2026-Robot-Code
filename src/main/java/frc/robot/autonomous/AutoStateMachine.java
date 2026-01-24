// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import frc.lib.team2930.AllianceFlipUtil;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.StateMachine;
import frc.lib.team6328.GeomUtil;
import frc.robot.Constants;
import frc.robot.RobotStates;
import frc.robot.autonomous.records.ChoreoTrajectoryWithName;
import frc.robot.autonomous.records.OppositeSide;
import frc.robot.autonomous.records.PickupLocation;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import java.util.ArrayList;
import java.util.List;

public class AutoStateMachine extends StateMachine {

  private final LED led;

  private final DrivetrainWrapper wrapper;
  private final RobotStates states;
  private final Intake intake;

  private final List<ChoreoTrajectoryWithName> scoringPaths = new ArrayList<>();
  private final List<ChoreoTrajectoryWithName> coralStationPaths = new ArrayList<>();

  private final RobotConfig config;

  private static LoggerGroup logGroup = LoggerGroup.build("AutoStateMachine");

  private static LoggerEntry.Struct<Pose3d> log_usedCoralTag =
      logGroup.buildStruct(Pose3d.class, "UsedCoralTag");
  private static LoggerEntry.Bool log_foundPath = logGroup.buildBoolean("PathFound");
  private static LoggerEntry.Struct<Pose2d> log_startingLocation =
      logGroup.buildStruct(Pose2d.class, "StartigLocation");

  private boolean preloadCode;

  /** Creates a new AutoSubstateMachine. */
  public AutoStateMachine(
      AutosSubsystems subsystems,
      RobotConfig config,
      boolean flipAuto,
      boolean procedural,
      RobotStates states) {
    super("Auto");

    led = subsystems.led();
    intake = subsystems.intake();
    wrapper = subsystems.drivetrain();
    this.states = states;

    boolean foundPath = scoringPaths.indexOf(null) == -1 && coralStationPaths.indexOf(null) == -1;

    log_foundPath.info(foundPath);

    if (foundPath) {
      for (int i = 0; i < scoringPaths.size(); i++) {
        scoringPaths.set(i, scoringPaths.get(i).flipOnAlliance(flipAuto).flipForAlliance());
      }
      for (int i = 0; i < coralStationPaths.size(); i++) {
        coralStationPaths.set(
            i, coralStationPaths.get(i).flipOnAlliance(flipAuto).flipForAlliance());
      }
    }

    this.config = config;
  }

  private static <T1, T2> ChoreoTrajectoryWithName enumsToPath(
      OppositeSide<T1> starting, OppositeSide<T2> ending) {
    ChoreoTrajectoryWithName traj = stringsToPathSimple(starting, ending);
    if (traj == null) {
      var oppositeStarting = starting.getOpposite();
      var oppositeEnding = ending.getOpposite();
      traj = stringsToPathSimple(oppositeStarting, oppositeEnding);
      if (traj == null) {
        throw new RuntimeException("Could not find " + starting + "_" + ending);
      }
      traj = traj.flipOnAlliance(true);
    }

    return traj;
  }

  private static ChoreoTrajectoryWithName stringsToPathSimple(
      OppositeSide<?> startString, OppositeSide<?> endString) {
    String fullString = startString.toString() + "_" + endString.toString();
    return ChoreoTrajectoryWithName.getTrajectory(fullString);
  }

  public Pose2d initPose() {
    ChoreoTrajectoryWithName traj = scoringPaths.get(0);
    if (traj == null) return null;
    return traj.getInitialPose(false);
  }

  private Pose2d getCoralStationPose(PickupLocation location) {
    boolean top = location == PickupLocation.IA || location == PickupLocation.IB;
    Pose3d tagPose = config.getAprilTagFieldLayout().getTagPose(top ? 13 : 12).get();
    log_usedCoralTag.info(tagPose);
    Pose2d pose = tagPose.toPose2d();
    Pose2d centerPickup =
        new Pose2d(
            pose.getTranslation()
                .plus(
                    new Translation2d(
                        Constants.RobotDimensions.ROBOT_DIMENSIONS_WITH_BUMPERS.getX() / 2.0,
                        pose.getRotation())),
            pose.getRotation().plus(Rotation2d.k180deg));

    boolean left = location == PickupLocation.IA || location == PickupLocation.IC;

    Pose2d offsetPickup =
        centerPickup.plus(
            GeomUtil.translationToTransform(
                new Translation2d(
                    Constants.FieldConstants.CORAL_STATION_WIDTH.div(4).in(Units.Meter),
                    left ? Rotation2d.kCW_90deg : Rotation2d.kCCW_90deg)));
    return AllianceFlipUtil.flipPoseForAlliance(offsetPickup);
  }
}
