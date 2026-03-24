package frc.robot.autonomous;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import frc.lib.team2930.*;
import frc.robot.RobotStates;
import frc.robot.autonomous.records.ChoreoVars;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class AutosManager {
  public enum Autos {
    NOTHING, TEST, OUTPOST_SIDE_SHOOT, OUTPOST_SIDE_SHOOT_PLUS, DEPOT, DEPOT_MIDDLE_DEPOTSIDE, MIDDLE_DEPOTSIDE,
    SWERVE_CHARACTERIZATION;
  }

  public enum AutoGoals {
    NOTHING, DRIVE, INTAKEWHILEDRIVELEFT, INTAKEWHILEDRIVERIGHT, SCORE, CLIMB;
  }

  private String ROOT_TABLE = "Sysid";

  public static class Waypoint {
    public final double                  time;
    public final AllianceAwarePose2d     pose;
    public final AllianceAwareVelocity2d velocity;
    public final AutoGoals               goal;
    public int                           sequence;

    public Waypoint(double time, Pose2d waypointPose, Translation2d velocity, AutoGoals goal) {
      this.time     = time;
      this.pose     = waypointPose != null ? new AllianceAwarePose2d(waypointPose) : null;
      this.velocity = velocity != null ? new AllianceAwareVelocity2d(velocity) : null;
      this.goal     = goal;
    }
  }

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
    list.add(this::middleOutpostSide);

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

  public AutoProgram goFast() {
    var autoPoints = createAutoPoints(Autos.TEST);
    var state      = new AutoStateMachine(subsystems, autoPoints.getFirst(), autoPoints.getSecond(), false, states);
    return stateToAutoProgram(state, autoPoints.getSecond(), "test go fast");
  }

  public AutoProgram depot() {
    var autoPoints = createAutoPoints(Autos.DEPOT);

    var state = new AutoStateMachine(subsystems, autoPoints.getFirst(), autoPoints.getSecond(), false, states);
    return stateToAutoProgram(state, autoPoints.getSecond(), "DEPOT");
  }

  public AutoProgram depotMiddle() {
    var autoPoints = createAutoPoints(Autos.DEPOT_MIDDLE_DEPOTSIDE);

    var state = new AutoStateMachine(subsystems, autoPoints.getFirst(), autoPoints.getSecond(), false, states);
    return stateToAutoProgram(state, autoPoints.getSecond(), "DEPOT MIDDLE DEPOT SIDE");
  }

  public AutoProgram middleDepotSide() {
    var autoPoints = createAutoPoints(Autos.MIDDLE_DEPOTSIDE);

    var state = new AutoStateMachine(subsystems, autoPoints.getFirst(), autoPoints.getSecond(), false, states);
    return stateToAutoProgram(state, autoPoints.getSecond(), "MIDDLE DEPOT SIDE");
  }

  public AutoProgram middleOutpostSide() {
    var autoPoints = createAutoPoints(Autos.OUTPOST_SIDE_SHOOT_PLUS);
    var state      = new AutoStateMachine(subsystems, autoPoints.getFirst(), autoPoints.getSecond(), false, states);
    return stateToAutoProgram(state, autoPoints.getSecond(), "MIDDLE OUTPOST SIDE PLUS");
  }

  private AllianceAwarePose2d newInitialPose(Pose2d pose) {
    return new AllianceAwarePose2d(pose);
  }

  private Waypoint newWaypoint(double time, Pose2d pose, Translation2d velocity, AutoGoals goals) {
    return new Waypoint(time, pose, velocity, goals);
  }

  private Pair<ArrayList<Waypoint>, AllianceAwarePose2d> createAutoPoints(Autos auto) {
    var                 waypoints = new ArrayList<Waypoint>();
    AllianceAwarePose2d aInitialPose;
    // @formatter:off
    switch (auto) {
      case TEST:
        aInitialPose = newInitialPose(ChoreoVars.Poses.START4);
        waypoints.add(newWaypoint(1.2, ChoreoVars.Poses.TEST_OVER_BUMP,  new Translation2d(0.0, 0.1), AutoGoals.DRIVE));
        waypoints.add(newWaypoint(2.5, ChoreoVars.Poses.TEST_MIDDLE_HUB,  new Translation2d(0, -0.5), AutoGoals.INTAKEWHILEDRIVERIGHT));
        waypoints.add(newWaypoint(1.2, ChoreoVars.Poses.TEST_OVER_BUMP,  new Translation2d(-0.5, 0), AutoGoals.DRIVE));
        waypoints.add(newWaypoint(1.3, ChoreoVars.Poses.SHOOT_OUTPOST_SIDE, null, AutoGoals.DRIVE));
        waypoints.add(newWaypoint(3.0, ChoreoVars.Poses.SHOOT_OUTPOST_SIDE, null, AutoGoals.SCORE));
        // waypoints.add(newWaypoint(6, ChoreoVars.Poses.SHOOT_OUTPOST_SIDE, null, AutoGoals.DRIVE));
        // waypoints.add(newWaypoint(7, ChoreoVars.Poses.SHOOT_OUTPOST_SIDE, null, AutoGoals.SCORE));
        return Pair.of(waypoints, aInitialPose);

      case OUTPOST_SIDE_SHOOT:
        aInitialPose = newInitialPose(ChoreoVars.Poses.START3);
        waypoints.add(newWaypoint(0, null, null, AutoGoals.SCORE));
        waypoints.add(newWaypoint(2, ChoreoVars.Poses.START3, null, AutoGoals.SCORE));
        return Pair.of(waypoints, aInitialPose);

      case OUTPOST_SIDE_SHOOT_PLUS:
        aInitialPose = newInitialPose(ChoreoVars.Poses.START3);
        waypoints.add(newWaypoint(0, null, null, AutoGoals.SCORE));
        waypoints.add(newWaypoint(2, ChoreoVars.Poses.START3, null, AutoGoals.DRIVE));
        waypoints.add(newWaypoint(2, ChoreoVars.Poses.INTAKE_PREP_MIDDLE_OUTPOSTSIDE, null, AutoGoals.DRIVE));
        waypoints.add(newWaypoint(2, ChoreoVars.Poses.INTAKE_MIDDLE_OUTPOSTSIDE, null, AutoGoals.INTAKEWHILEDRIVERIGHT));
        waypoints.add(newWaypoint(1, ChoreoVars.Poses.INTAKE_PREP_MIDDLE_MIDDLE_OUTPOSTSIDE, null, AutoGoals.INTAKEWHILEDRIVERIGHT));
        waypoints.add(newWaypoint(2, ChoreoVars.Poses.INTAKE_MIDDLE_MIDDLE_OUTPOSTSIDE, null, AutoGoals.INTAKEWHILEDRIVERIGHT));
        waypoints.add(newWaypoint(2, ChoreoVars.Poses.INTAKE_PREP_MIDDLE_OUTPOSTSIDE, null, AutoGoals.DRIVE));
        waypoints.add(newWaypoint(2, ChoreoVars.Poses.SHOOT_OUTPOST_SIDE, null, AutoGoals.DRIVE));
        waypoints.add(newWaypoint(2, ChoreoVars.Poses.SHOOT_OUTPOST_SIDE, null, AutoGoals.SCORE));
        return Pair.of(waypoints, aInitialPose);

      case DEPOT:
        aInitialPose = newInitialPose(ChoreoVars.Poses.START1);
        waypoints.add(newWaypoint(2, ChoreoVars.Poses.DEPOT_PREP, new Translation2d(-0.5, 0), AutoGoals.DRIVE));
        waypoints.add(newWaypoint(1, ChoreoVars.Poses.DEPOT, null, AutoGoals.INTAKEWHILEDRIVERIGHT));
        waypoints.add(newWaypoint(2, ChoreoVars.Poses.SHOOT_DEPOT_SIDE, null, AutoGoals.DRIVE));
        waypoints.add(newWaypoint(2, ChoreoVars.Poses.SHOOT_DEPOT_SIDE, null, AutoGoals.SCORE));
        waypoints.add(newWaypoint(2, ChoreoVars.Poses.SHOOT_DEPOT_SIDE, null, AutoGoals.DRIVE));
        return Pair.of(waypoints, aInitialPose);

      case DEPOT_MIDDLE_DEPOTSIDE:
        aInitialPose = newInitialPose(ChoreoVars.Poses.START1);
        waypoints.add(newWaypoint(0, null, null, AutoGoals.SCORE));
        waypoints.add(newWaypoint(2, ChoreoVars.Poses.START1, null, AutoGoals.DRIVE));
        waypoints.add(newWaypoint(2, ChoreoVars.Poses.DEPOT_PREP, null, AutoGoals.DRIVE));
        waypoints.add(newWaypoint(1, ChoreoVars.Poses.DEPOT, null, AutoGoals.INTAKEWHILEDRIVERIGHT));
        waypoints.add(newWaypoint(1, ChoreoVars.Poses.SHOOT_DEPOT_SIDE, null, AutoGoals.DRIVE));
        waypoints.add(newWaypoint(2, ChoreoVars.Poses.SHOOT_DEPOT_SIDE, null, AutoGoals.SCORE));
        waypoints.add(newWaypoint(1, ChoreoVars.Poses.INTAKE_PREP_MIDDLE_DEPOTSIDE, null, AutoGoals.DRIVE));
        waypoints.add(newWaypoint(1, ChoreoVars.Poses.INTAKE_MIDDLE_DEPOTSIDE, null, AutoGoals.INTAKEWHILEDRIVELEFT));
        waypoints.add(newWaypoint(1, ChoreoVars.Poses.INTAKE_PREP_MIDDLE_MIDDLE_DEPOTSIDE, null, AutoGoals.INTAKEWHILEDRIVELEFT));
        waypoints.add(newWaypoint(1, ChoreoVars.Poses.INTAKE_MIDDLE_MIDDLE_DEPOTSIDE, null, AutoGoals.INTAKEWHILEDRIVELEFT));
        waypoints.add(newWaypoint(1, ChoreoVars.Poses.BUMP_PREP_MIDDLE_DEPOTSIDE, null, AutoGoals.DRIVE));
        waypoints.add(newWaypoint(1, ChoreoVars.Poses.SHOOT_DEPOT_SIDE, null, AutoGoals.DRIVE));
        waypoints.add(newWaypoint(1, ChoreoVars.Poses.SHOOT_DEPOT_SIDE, null, AutoGoals.SCORE));
        return Pair.of(waypoints, aInitialPose);

      case MIDDLE_DEPOTSIDE:
        aInitialPose = newInitialPose(ChoreoVars.Poses.START1);
        waypoints.add(newWaypoint(0, null, null, AutoGoals.SCORE));
        waypoints.add(newWaypoint(3.0, ChoreoVars.Poses.START1, null, AutoGoals.DRIVE));
        waypoints.add(newWaypoint(0.6, ChoreoVars.Poses.RIGHT_OVER_BUMP_DEPOTSIDE, null, AutoGoals.DRIVE));
        waypoints.add(newWaypoint(0.3, ChoreoVars.Poses.NEAR_TRENCH_ANGLED_DEPOTSIDE, null, AutoGoals.DRIVE));
        waypoints.add(newWaypoint(0.3, ChoreoVars.Poses.NEAR_TRENCH_DEPOTSIDE, null, AutoGoals.DRIVE));
        waypoints.add(newWaypoint(0.3, ChoreoVars.Poses.INTAKE_PREP_DEPOTSIDE, null, AutoGoals.DRIVE));
        waypoints.add(newWaypoint(0.7, ChoreoVars.Poses.INTAKE_MIDDLE_DEPOTSIDE, null, AutoGoals.INTAKEWHILEDRIVELEFT));
        waypoints.add(newWaypoint(3, ChoreoVars.Poses.INTAKE_MIDDLE_MIDDLE_DEPOTSIDE, null, AutoGoals.INTAKEWHILEDRIVELEFT));
        waypoints.add(newWaypoint(0.4, ChoreoVars.Poses.BUMP_PREP_MIDDLE_DEPOTSIDE, null, AutoGoals.DRIVE));
        waypoints.add(newWaypoint(0.7, ChoreoVars.Poses.SHOOT_DEPOT_SIDE, null, AutoGoals.DRIVE));
        waypoints.add(newWaypoint(2.5, ChoreoVars.Poses.SHOOT_DEPOT_SIDE, null, AutoGoals.SCORE));
        waypoints.add(newWaypoint(0.6, ChoreoVars.Poses.RIGHT_OVER_BUMP_DEPOTSIDE, null, AutoGoals.DRIVE));
        waypoints.add(newWaypoint(0.3, ChoreoVars.Poses.NEAR_TRENCH_ANGLED_DEPOTSIDE, null, AutoGoals.DRIVE));
        waypoints.add(newWaypoint(0.3, ChoreoVars.Poses.NEAR_TRENCH_DEPOTSIDE, null, AutoGoals.INTAKEWHILEDRIVELEFT));
        waypoints.add(newWaypoint(0.3, ChoreoVars.Poses.INTAKE_PREP_DEPOTSIDE, null, AutoGoals.INTAKEWHILEDRIVELEFT));
        waypoints.add(newWaypoint(0.7, ChoreoVars.Poses.INTAKE_MIDDLE_DEPOTSIDE, null, AutoGoals.INTAKEWHILEDRIVELEFT));
        waypoints.add(newWaypoint(2.4, ChoreoVars.Poses.INTAKE_MIDDLE_MIDDLE_DEPOTSIDE, null, AutoGoals.INTAKEWHILEDRIVELEFT));
        waypoints.add(newWaypoint(0.4, ChoreoVars.Poses.BUMP_PREP_MIDDLE_DEPOTSIDE, null, AutoGoals.DRIVE));
        waypoints.add(newWaypoint(0.7, ChoreoVars.Poses.SHOOT_DEPOT_SIDE, null, AutoGoals.DRIVE));
        waypoints.add(newWaypoint(2.5, ChoreoVars.Poses.SHOOT_DEPOT_SIDE, null, AutoGoals.SCORE));
        return Pair.of(waypoints, aInitialPose);

      default:
        aInitialPose = newInitialPose(Pose2d.kZero);
        return Pair.of(new ArrayList<>(), aInitialPose);
    }
    // @formatter:on
  }

  private AutoProgram stateToAutoProgram(AutoStateMachine state, AllianceAwarePose2d initialPose, String name) {
    return new AutoProgram(state.asCommand(), initialPose, name);
  }
}
