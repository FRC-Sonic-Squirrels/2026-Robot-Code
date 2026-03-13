package frc.robot.autonomous;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import frc.lib.team2930.AllianceAwarePose2d;
import frc.lib.team2930.AutoRegisterLoggers;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.robot.RobotStates;
import frc.robot.autonomous.records.ChoreoVars;
import frc.robot.configs.RobotConfig;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutosManager {
  public enum Autos {
    NOTHING, TEST, DEPOT, DEPOTPLUS, MIDDLE_DEPOTSIDE, SWERVE_CHARACTERIZATION;
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
  private final RobotConfig     config;
  private final RobotStates     states;
  private final BooleanSupplier flipAuto;

  public final boolean includeDebugPaths = false;

  private boolean preloadingComplete = !false;

  public AutosManager(AutosSubsystems subsystems, RobotConfig config, LoggedDashboardChooser<String> chooser,
      HashMap<String, Supplier<AutoProgram>> stringToAutoSupplierMap, BooleanSupplier flipAuto, RobotStates states) {
    this.subsystems = subsystems;
    this.config     = config;
    this.flipAuto   = flipAuto;
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
    list.add(this::depot);

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

  public AutoProgram depot() {
    Pair<ArrayList<Waypoint>, AllianceAwarePose2d> autoPoints = createAutoPoints(Autos.DEPOT);
    var                                            state      = new AutoStateMachine(subsystems, autoPoints.getFirst(),
        autoPoints.getSecond(), states);
    return stateToAutoProgram(state, autoPoints.getSecond(), "DEPOT");
  }

  // @SuppressWarnings("unused")
  // private AutoProgram testPath(String pathName, boolean useInitialPose) {
  // return testPath(pathName, useInitialPose, pathName);
  // }

  // TODO: Potentially get rid of testPath? See if it's necessary.
  // private AutoProgram testPath(String pathName, boolean useInitialPose, String
  // autoName) {
  // var traj = ChoreoTrajectoryWithName.getTrajectory(pathName);
  // return new AutoProgram(Pair.of(new Command() {
  // private final DrivetrainWrapper drivetrain = subsystems.drivetrain();
  // ChoreoHelper helper;
  // boolean atEndOfPath;

  // @Override
  // public void initialize() {
  // drivetrain.setPose(Constants.zeroPose2d);
  // helper = new ChoreoHelper(Timer.getFPGATimestamp(),
  // drivetrain.getHubPoseEstimatorPose(true), traj,
  // config.getDriveBaseRadius() / 2, config.getAutoTranslationPidController(),
  // config.getAutoTranslationPidController(),
  // config.getAutoThetaPidController());
  // }

  // @Override
  // public void execute() {
  // var result =
  // helper.calculateChassisSpeeds(drivetrain.getHubPoseEstimatorPose(true),
  // Timer.getFPGATimestamp());
  // drivetrain.setVelocityOverride(result.chassisSpeeds());
  // atEndOfPath = result.atEndOfPath();
  // }

  // @Override
  // public void end(boolean interrupted) {
  // drivetrain.resetVelocityOverride();
  // }

  // @Override
  // public boolean isFinished() {
  // return atEndOfPath;
  // }
  // }, Constants.zeroPose2d), autoName);
  // }

  // TODO: Finish up swerveCharacterization to reinstate it
  // public AutoProgram swerveCharacterization() {
  // var sysidConfig = new SysIdRoutine.Config(null, null, null, (state) -> {
  // if (state != SysIdRoutineLog.State.kNone) {
  // logs.swervesysidstate.info(state);
  // }
  // });

  // var mechanism = new SysIdRoutine(sysidConfig,
  // subsystems.drivetrain().getSysIdMechanism());

  // var command1 = mechanism.quasistatic(Direction.kForward);
  // var command2 = mechanism.quasistatic(Direction.kReverse);
  // var command3 = mechanism.dynamic(Direction.kForward);
  // var command4 = mechanism.dynamic(Direction.kReverse);

  // var finalCommand = command1
  // // .andThen(Commands.waitSeconds(1.0))
  // .andThen(command2)
  // // .andThen(Commands.waitSeconds(1.0))
  // .andThen(command3);
  // // .andThen(Commands.waitSeconds(1.0))
  // // .andThen(command4).andThen(Commands.runOnce(() ->
  // // logSwerveSysidState.info(SysIdRoutineLog.State.kNone)));
  // // return new AutoProgram(null, Pose2d.kZero, "Swerve Characterization");
  // }

  public boolean preloadingComplete() {
    return preloadingComplete;
  }

  private Pair<ArrayList<Waypoint>, AllianceAwarePose2d> createAutoPoints(Autos auto) {
    ArrayList<Waypoint> waypoints = new ArrayList<Waypoint>();
    AllianceAwarePose2d aInitialPose;
    switch (auto) {
      case DEPOT:
        aInitialPose = new AllianceAwarePose2d(ChoreoVars.Poses.START1);
        waypoints.add(new Waypoint(ChoreoVars.Poses.DEPOT_PREP, AutoGoals.DRIVE, false));
        waypoints.add(new Waypoint(ChoreoVars.Poses.DEPOT, AutoGoals.INTAKEWHILEDRIVERIGHT, true));
        waypoints.add(new Waypoint(ChoreoVars.Poses.SHOOT_DEPOT_SIDE, AutoGoals.DRIVE, false));
        waypoints.add(new Waypoint(ChoreoVars.Poses.SHOOT_DEPOT_SIDE, AutoGoals.SCORE, false));
        return Pair.of(waypoints, aInitialPose);

      case MIDDLE_DEPOTSIDE:
        aInitialPose = new AllianceAwarePose2d(ChoreoVars.Poses.START2);
        waypoints.add(new Waypoint(ChoreoVars.Poses.INTAKE_PREP_MIDDLE_DEPOTSIDE, AutoGoals.DRIVE, false));
        waypoints.add(new Waypoint(ChoreoVars.Poses.INTAKE_MIDDLE_DEPOTSIDE, AutoGoals.INTAKEWHILEDRIVELEFT, true));
        waypoints.add(
            new Waypoint(ChoreoVars.Poses.INTAKE_PREP_MIDDLE_MIDDLE_DEPOTSIDE, AutoGoals.INTAKEWHILEDRIVELEFT, false));
        waypoints
            .add(new Waypoint(ChoreoVars.Poses.INTAKE_MIDDLE_MIDDLE_DEPOTSIDE, AutoGoals.INTAKEWHILEDRIVELEFT, true));
        waypoints.add(new Waypoint(ChoreoVars.Poses.BUMP_PREP_MIDDLE_DEPOTSIDE, AutoGoals.DRIVE, false));
        waypoints.add(new Waypoint(ChoreoVars.Poses.SHOOT_DEPOT_SIDE, AutoGoals.DRIVE, false));
        waypoints.add(new Waypoint(ChoreoVars.Poses.SHOOT_DEPOT_SIDE, AutoGoals.SCORE, false));

      default:
        aInitialPose = new AllianceAwarePose2d(Pose2d.kZero);
        return Pair.of(new ArrayList<>(), aInitialPose);
    }
  }

  private AutoProgram stateToAutoProgram(AutoStateMachine state, AllianceAwarePose2d initialPose, String name) {
    return new AutoProgram(state.asCommand(), initialPose, name);
  }
}
