package frc.robot.autonomous;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import frc.lib.team2930.*;
import frc.robot.RobotStates;
import frc.robot.autonomous.records.ChoreoVars;

import java.nio.channels.NonWritableChannelException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutosManager {
  public enum Autos {
    NOTHING, TEST, DEPOT, DEPOT_MIDDLE_DEPOTSIDE, MIDDLE_DEPOTSIDE, SWERVE_CHARACTERIZATION;
  }

  public enum AutoGoals {
    NOTHING, DRIVE, INTAKEWHILEDRIVELEFT, INTAKEWHILEDRIVERIGHT, SCORE, CLIMB;
  }

  public record Waypoint(AllianceAwarePose2d waypointPose, AutoGoals goal, boolean goSlow) {
    public Waypoint(Pose2d waypointPose, AutoGoals goal, boolean goSlow) {
      this(new AllianceAwarePose2d(waypointPose), goal, goSlow);
    }
  }

  private String ROOT_TABLE = "Sysid";

  public static class Logs extends AutoRegisterLoggers {
    public LoggerEntry.EnumValue<SysIdRoutineLog.State> swervesysidstate;
  }

  private final Logs            logs = LoggerGroup.root.newAutoLogger(Logs.class, ROOT_TABLE);
  private final AutosSubsystems subsystems;
  private final RobotStates     states;

  public final boolean includeDebugPaths = false;

  public AutosManager(AutosSubsystems subsystems, LoggedDashboardChooser<String> chooser,
      HashMap<String, Supplier<AutoProgram>> stringToAutoSupplierMap, BooleanSupplier flipAuto, RobotStates states) {
    this.subsystems = subsystems;
    this.states     = states;
    // preload code
    // Command preloader;
    fillChooserAndMap(chooser, stringToAutoSupplierMap);
    // TODO: Make a decision about the necessity of the preloader.
    // var cmd =
    // Commands.waitSeconds(6)
    // .deadlineFor(preloader.repeatedly())
    // .andThen(Commands.runOnce(() -> preloadingComplete = true))
    // .ignoringDisable(true)
    // .withName("AutoPreload");
    // CommandScheduler.getInstance().schedule(cmd);

  }

  private List<Supplier<AutoProgram>> allCompetitionAutos() {
    var list = new ArrayList<Supplier<AutoProgram>>();

    list.add(this::doNothing);
    list.add(this::goFast);
    list.add(this::depot);
    list.add(this::depotMiddle);
    list.add(this::middleDepotSide);

    if (includeDebugPaths) {
      // list.add(this::swerveCharacterization);
      // list.add(() -> testPath("TestDrive1Meter", true));
      // list.add(() -> testPath("TestDrive10Meter", true));
      // list.add(() -> testPath("TestDrive2Meters", true));
      // list.add(() -> testPath("TestDrive2MetersRotating", true));
      // list.add(() -> testPath("TestDrive2MetersThenLeft", true));
      // list.add(() -> testPath("TestDrive2MetersThenLeftRotating", true));
      // list.add(() -> testPath("TestCircle", true));
      // list.add(() -> testPath("TestCircle", false, "TestCircleDontResetPose"));
      // list.add(() -> testPath("TestZigZag", false));
    }

    return list;
  }

  private void fillChooserAndMap(LoggedDashboardChooser<String> chooser,
      HashMap<String, Supplier<AutoProgram>> stringToAutoSupplierMap) {
    var compAutos = allCompetitionAutos();

    for (int i = 0; i < compAutos.size(); i++) {
      var supplier = compAutos.get(i);
      var name     = supplier.get().getName();
      if (i == 0) {
        // Do nothing command must be first in list.
        chooser.addDefaultOption(name, name);
      } else {
        chooser.addOption(name, name);
      }

      stringToAutoSupplierMap.put(name, supplier);
    }
  }

  public AutoProgram doNothing() {
    return new AutoProgram(null, new AllianceAwarePose2d(Pose2d.kZero), "Do Nothing");
  }

  public AutoProgram goFast(){
    var autoPoints = createAutoPoints(Autos.TEST);
    var state = new AutoStateMachine(subsystems, autoPoints.getFirst(), autoPoints.getSecond(), states);
    return stateToAutoProgram(state, autoPoints.getSecond(), "test go fast");
  }  

  public AutoProgram depot() {
    var autoPoints = createAutoPoints(Autos.DEPOT);

    var state = new AutoStateMachine(subsystems, autoPoints.getFirst(), autoPoints.getSecond(), states);
    return stateToAutoProgram(state, autoPoints.getSecond(), "DEPOT");
  }

  public AutoProgram depotMiddle() {
    var autoPoints = createAutoPoints(Autos.DEPOT_MIDDLE_DEPOTSIDE);

    var state = new AutoStateMachine(subsystems, autoPoints.getFirst(), autoPoints.getSecond(), states);
    return stateToAutoProgram(state, autoPoints.getSecond(), "DEPOT MIDDLE DEPOT SIDE");
  }

  public AutoProgram middleDepotSide() {
    var autoPoints = createAutoPoints(Autos.MIDDLE_DEPOTSIDE);

    var state = new AutoStateMachine(subsystems, autoPoints.getFirst(), autoPoints.getSecond(), states);
    return stateToAutoProgram(state, autoPoints.getSecond(), "MIDDLE DEPOT SIDE");
  }

  private AllianceAwarePose2d newInitialPose(Pose2d pose) {
    return new AllianceAwarePose2d(fixPose(pose));
  }

  private Pose2d fixPose(Pose2d pose) {
    return GeometryUtil.rotateIfNeeded(pose, states.config.isRobotRotatedBackward());
  }

  private Waypoint newWaypoint(Pose2d pose, AutoGoals goals, boolean goSlow) {
    return new Waypoint(fixPose(pose), goals, goSlow);
  }

  private Pair<ArrayList<Waypoint>, AllianceAwarePose2d> createAutoPoints(Autos auto) {
    var                 waypoints = new ArrayList<Waypoint>();
    AllianceAwarePose2d aInitialPose;
    switch (auto) {
      case TEST:
        aInitialPose = newInitialPose(ChoreoVars.Poses.START4);
        waypoints.add(newWaypoint(ChoreoVars.Poses.TEST_OVER_BUMP, AutoGoals.DRIVE, false));
        waypoints.add(newWaypoint(ChoreoVars.Poses.TEST_MIDDLE_HUB, AutoGoals.DRIVE, false));
        waypoints.add(newWaypoint(ChoreoVars.Poses.TEST_OVER_BUMP, AutoGoals.DRIVE, false));
        waypoints.add(newWaypoint(ChoreoVars.Poses.SHOOT_OUTPOST_SIDE, AutoGoals.DRIVE, false));
        //waypoints.add(newWaypoint(ChoreoVars.Poses.SHOOT_OUTPOST_SIDE, AutoGoals.DRIVE, false));
        //waypoints.add(newWaypoint(ChoreoVars.Poses.SHOOT_OUTPOST_SIDE, AutoGoals.SCORE, false));
        return Pair.of(waypoints, aInitialPose);

      case DEPOT:
        aInitialPose = newInitialPose(ChoreoVars.Poses.START1);
        waypoints.add(newWaypoint(ChoreoVars.Poses.DEPOT_PREP, AutoGoals.DRIVE, false));
        waypoints.add(newWaypoint(ChoreoVars.Poses.DEPOT, AutoGoals.INTAKEWHILEDRIVERIGHT, true));
        waypoints.add(newWaypoint(ChoreoVars.Poses.SHOOT_DEPOT_SIDE, AutoGoals.DRIVE, false));
        waypoints.add(newWaypoint(ChoreoVars.Poses.SHOOT_DEPOT_SIDE, AutoGoals.SCORE, false));
        return Pair.of(waypoints, aInitialPose);

      case DEPOT_MIDDLE_DEPOTSIDE:
        aInitialPose = newInitialPose(ChoreoVars.Poses.START1);
        waypoints.add(newWaypoint(ChoreoVars.Poses.DEPOT_PREP, AutoGoals.DRIVE, false));
        waypoints.add(newWaypoint(ChoreoVars.Poses.DEPOT, AutoGoals.INTAKEWHILEDRIVERIGHT, true));
        waypoints.add(newWaypoint(ChoreoVars.Poses.SHOOT_DEPOT_SIDE, AutoGoals.DRIVE, false));
        waypoints.add(newWaypoint(ChoreoVars.Poses.SHOOT_DEPOT_SIDE, AutoGoals.SCORE, false));
        waypoints.add(newWaypoint(ChoreoVars.Poses.INTAKE_PREP_MIDDLE_DEPOTSIDE, AutoGoals.DRIVE, false));
        waypoints.add(newWaypoint(ChoreoVars.Poses.INTAKE_MIDDLE_DEPOTSIDE, AutoGoals.INTAKEWHILEDRIVELEFT, true));
        waypoints.add(
            newWaypoint(ChoreoVars.Poses.INTAKE_PREP_MIDDLE_MIDDLE_DEPOTSIDE, AutoGoals.INTAKEWHILEDRIVELEFT, false));
        waypoints
            .add(newWaypoint(ChoreoVars.Poses.INTAKE_MIDDLE_MIDDLE_DEPOTSIDE, AutoGoals.INTAKEWHILEDRIVELEFT, true));
        waypoints.add(newWaypoint(ChoreoVars.Poses.BUMP_PREP_MIDDLE_DEPOTSIDE, AutoGoals.DRIVE, false));
        waypoints.add(newWaypoint(ChoreoVars.Poses.SHOOT_DEPOT_SIDE, AutoGoals.DRIVE, false));
        waypoints.add(newWaypoint(ChoreoVars.Poses.SHOOT_DEPOT_SIDE, AutoGoals.SCORE, false));
        return Pair.of(waypoints, aInitialPose);

      case MIDDLE_DEPOTSIDE:
        aInitialPose = newInitialPose(ChoreoVars.Poses.START2);
        waypoints.add(newWaypoint(ChoreoVars.Poses.INTAKE_PREP_MIDDLE_DEPOTSIDE, AutoGoals.DRIVE, false));
        waypoints.add(newWaypoint(ChoreoVars.Poses.INTAKE_MIDDLE_DEPOTSIDE, AutoGoals.INTAKEWHILEDRIVELEFT, true));
        waypoints.add(
            newWaypoint(ChoreoVars.Poses.INTAKE_PREP_MIDDLE_MIDDLE_DEPOTSIDE, AutoGoals.INTAKEWHILEDRIVELEFT, false));
        waypoints
            .add(newWaypoint(ChoreoVars.Poses.INTAKE_MIDDLE_MIDDLE_DEPOTSIDE, AutoGoals.INTAKEWHILEDRIVELEFT, true));
        waypoints.add(newWaypoint(ChoreoVars.Poses.BUMP_PREP_MIDDLE_DEPOTSIDE, AutoGoals.DRIVE, false));
        waypoints.add(newWaypoint(ChoreoVars.Poses.SHOOT_DEPOT_SIDE, AutoGoals.DRIVE, false));
        waypoints.add(newWaypoint(ChoreoVars.Poses.SHOOT_DEPOT_SIDE, AutoGoals.SCORE, false));
        return Pair.of(waypoints, aInitialPose);

      default:
        aInitialPose = newInitialPose(Pose2d.kZero);
        return Pair.of(new ArrayList<>(), aInitialPose);
    }
  }

  private AutoProgram stateToAutoProgram(AutoStateMachine state, AllianceAwarePose2d initialPose, String name) {
    return new AutoProgram(state.asCommand(), initialPose, name);
  }
}
