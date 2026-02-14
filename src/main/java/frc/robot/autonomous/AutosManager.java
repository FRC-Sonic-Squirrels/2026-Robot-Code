package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.RunStateMachineCommand;
import frc.robot.Constants;
import frc.robot.RobotStates;
import frc.robot.autonomous.helpers.ChoreoHelper;
import frc.robot.autonomous.records.ChoreoTrajectoryWithName;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.swerve.DrivetrainWrapper;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutosManager {
  private static final LoggerGroup                                  logGroupSysid       = LoggerGroup.build("Sysid");
  private static final LoggerEntry.EnumValue<SysIdRoutineLog.State> logSwerveSysidState = logGroupSysid
      .buildEnum("swervesysidstate");

  private final AutosSubsystems subsystems;
  private final RobotConfig     config;
  private final BooleanSupplier flipAuto;
  private final RobotStates     states;

  public final boolean includeDebugPaths = false;

  private boolean preloadingComplete = !false;

  public record Auto(String name, Command command, Pose2d initPose) {
  }

  public AutosManager(AutosSubsystems subsystems, RobotConfig config, LoggedDashboardChooser<String> chooser,
      HashMap<String, Supplier<Auto>> stringToAutoSupplierMap, BooleanSupplier flipAuto, RobotStates states) {
    this.subsystems = subsystems;
    this.config     = config;
    this.flipAuto   = flipAuto;
    this.states     = states;
    Command preloader = new RunStateMachineCommand(() -> new AutoStateMachine(subsystems, config, false, false, states),
        true);

    var cmd = Commands.waitSeconds(6).deadlineFor(preloader.repeatedly())
        .andThen(Commands.runOnce(() -> preloadingComplete = true)).ignoringDisable(true).withName("AutoPreload");
    CommandScheduler.getInstance().schedule(cmd);

    fillChooserAndMap(chooser, stringToAutoSupplierMap);
  }

  private List<Supplier<Auto>> allCompetitionAutos() {
    var list = new ArrayList<Supplier<Auto>>();
    list.add(this::doNothing);
    if (includeDebugPaths) {
      list.add(this::swerveCharacterization);
    }
    return list;
  }

  private void fillChooserAndMap(LoggedDashboardChooser<String> chooser,
      HashMap<String, Supplier<Auto>> stringToAutoSupplierMap) {
    var compAutos = allCompetitionAutos();
    for (int i = 0; i < compAutos.size(); i++) {
      var supplier = compAutos.get(i);
      var name     = supplier.get().name;
      if (i == 0) {
        chooser.addDefaultOption(name, name);
      } else {
        chooser.addOption(name, name);
      }
      stringToAutoSupplierMap.put(name, supplier);
    }
  }

  public Auto doNothing() {
    return new Auto("doNothing", new InstantCommand(), Constants.zeroPose2d);
  }

  @SuppressWarnings("unused")
  private Auto testPath(String pathName, boolean useInitialPose) {
    return testPath(pathName, useInitialPose, pathName);
  }

  private Auto testPath(String pathName, boolean useInitialPose, String autoName) {
    var traj = ChoreoTrajectoryWithName.getTrajectory(pathName);
    return new Auto(autoName, new Command() {
      private final DrivetrainWrapper drivetrain = subsystems.drivetrain();
      ChoreoHelper                    helper;
      boolean                         atEndOfPath;

      @Override
      public void initialize() {
        drivetrain.setPose(Constants.zeroPose2d);
        helper = new ChoreoHelper(Timer.getFPGATimestamp(), drivetrain.getHubPoseEstimatorPose(true), traj,
            config.getDriveBaseRadius() / 2, config.getAutoTranslationPidController(),
            config.getAutoTranslationPidController(), config.getAutoThetaPidController());
      }

      @Override
      public void execute() {
        var result = helper.calculateChassisSpeeds(drivetrain.getHubPoseEstimatorPose(true), Timer.getFPGATimestamp());
        drivetrain.setVelocityOverride(result.chassisSpeeds());
        atEndOfPath = result.atEndOfPath();
      }

      @Override
      public void end(boolean interrupted) {
        drivetrain.resetVelocityOverride();
      }

      @Override
      public boolean isFinished() {
        return atEndOfPath;
      }
    }, useInitialPose ? traj.getInitialPose(true) : null);
  }

  public Auto swerveCharacterization() {
    var sysidConfig = new SysIdRoutine.Config(null, null, null, (state) -> {
      if (state != SysIdRoutineLog.State.kNone) {
        logSwerveSysidState.info(state);
      }
    });

    var mechanism = new SysIdRoutine(sysidConfig, subsystems.drivetrain().getSysIdMechanism());

    var command1 = mechanism.quasistatic(Direction.kForward);
    var command2 = mechanism.quasistatic(Direction.kReverse);
    var command3 = mechanism.dynamic(Direction.kForward);
    var command4 = mechanism.dynamic(Direction.kReverse);

    var finalCommand = command1
        // .andThen(Commands.waitSeconds(1.0))
        .andThen(command2)
        // .andThen(Commands.waitSeconds(1.0))
        .andThen(command3)
        // .andThen(Commands.waitSeconds(1.0))
        .andThen(command4).andThen(Commands.runOnce(() -> logSwerveSysidState.info(SysIdRoutineLog.State.kNone)));

    return new Auto("swerveCharacterization", finalCommand, Constants.zeroPose2d);
  }

  private Auto stateToAuto(String name, AutoStateMachine state) {
    return new Auto(name, state.asCommand(), state.initPose());
  }

  public boolean preloadingComplete() {
    return preloadingComplete;
  }
}
