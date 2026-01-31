// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.*;
import frc.lib.team2930.commands.RunsWhenDisabledInstantCommand;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.RobotMode.Mode;
import frc.robot.Constants.RobotMode.RobotType;
import frc.robot.RobotStates.MechState;
import frc.robot.RobotStates.RobotGoal;
import frc.robot.autonomous.AutosManager;
import frc.robot.autonomous.AutosManager.Auto;
import frc.robot.autonomous.AutosSubsystems;
import frc.robot.autonomous.records.PickupLocation;
import frc.robot.commands.climber.ClimberSetAngle;
import frc.robot.commands.drive.DrivetrainDefaultTeleopDrive;
import frc.robot.commands.drive.WheelRadiusCharacterization;
import frc.robot.configs.SimulatorRobotConfig;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.BaseRobotState;
import frc.robot.subsystems.LED.RobotState;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOReal;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOPigeon2;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionModuleConfiguration;
import frc.robot.subsystems.visionGamepiece.VisionGamepiece;
import frc.robot.subsystems.visionGamepiece.VisionGamepieceIO;
import frc.robot.subsystems.visionGamepiece.VisionGamepieceIOReal;
import frc.robot.subsystems.visionGamepiece.VisionGamepieceIOSim;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Robot logging
  private static final LoggerEntry.EnumValue<RobotType> logRobotType =
      LoggerGroup.root.buildEnum("RobotType");
  private static final LoggerEntry.EnumValue<Mode> logRobotMode =
      LoggerGroup.root.buildEnum("RobotMode");

  // Subsystems
  private final Drivetrain drivetrain;
  private final DrivetrainWrapper drivetrainWrapper;
  private final AprilTagFieldLayout aprilTagLayout;
  private final Vision vision;
  private final Intake intake;
  private final VisionGamepiece visionGamepiece;
  private final LED led;
  private final Climber climber;

  // Controllers
  private final XboxControllerWrapper driverController = new XboxControllerWrapper(0);
  private final XboxControllerWrapper operatorController = new XboxControllerWrapper(1);

  // Auto
  private final LoggedDashboardChooser<String> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");
  private final LoggedDashboardChooser<Boolean> flipAutoChooser =
      new LoggedDashboardChooser<>("Flip Auto?");
  private final List<LoggedDashboardChooser<PickupLocation>> coralStationPosChooser =
      new ArrayList<>();
  private final int customGamepieceCount = 5;

  private final HashMap<String, Supplier<Auto>> stringToAutoSupplierMap = new HashMap<>();
  private final AutosManager autoManager;
  // Buttons

  public DigitalInput brakeModeButton = new DigitalInput(0);
  public DigitalInput zeroSensorsButton = new DigitalInput(1);

  private final Trigger brakeModeButtonTrigger =
      new Trigger(() -> !brakeModeButton.get() && !DriverStation.isEnabled());

  private final Trigger zeroSensorsButtonTrigger =
      new Trigger(() -> !zeroSensorsButton.get() && !DriverStation.isEnabled());

  private boolean brakeModeTriggered = true;
  private boolean zeroButtonTriggered = false;

  // Robot Modes
  @SuppressWarnings("unused")
  private boolean is_teleop;

  private boolean is_autonomous;

  private boolean brakeModeFailure = false;

  // Tunable numbers

  private final TunableNumberGroup tunableNumberGroup = new TunableNumberGroup("RobotContainer");

  @SuppressWarnings("unused")
  private final LoggedTunableNumber tunableX = tunableNumberGroup.build("TunableX", 13.75);

  @SuppressWarnings("unused")
  private final LoggedTunableNumber tunableY = tunableNumberGroup.build("TunableY", 5.15);

  @SuppressWarnings("unused")
  private final LoggedTunableNumber tunableAngle = tunableNumberGroup.build("TunableAngle", 0);

  private final RobotStates robotStates;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    robotStates = new RobotStates();

    DriverStation.silenceJoystickConnectionWarning(true); // Silence annoying joystick warning

    RobotType robotType = Constants.RobotMode.getRobot();
    Mode mode = Constants.RobotMode.getMode();

    logRobotType.info(robotType);
    logRobotMode.info(mode);

    var config = robotType.config.get();
    aprilTagLayout = config.getAprilTagFieldLayout();

    if (mode == Mode.REPLAY) {
      drivetrain =
          new Drivetrain(
              config,
              new GyroIO.Fake(),
              new GyroIO.Fake(),
              config.getReplaySwerveModuleObjects(),
              () -> is_autonomous);

      vision =
          new Vision(
              aprilTagLayout,
              drivetrain::getHubPoseEstimatorPose,
              drivetrain::getRotationGyroOnly,
              drivetrain::addVisionEstimate,
              config.getReplayVisionModules());

      intake = new Intake(new IntakeIO() {}, robotStates);
      climber = new Climber(new ClimberIO() {});

      if (Constants.unusedCode) {
        visionGamepiece =
            new VisionGamepiece(
                new VisionGamepieceIO() {}, drivetrain::getHubPoseEstimatorPoseAtTimestamp);
      } else {
        visionGamepiece = null;
      }

      led =
          new LED(
              () -> brakeModeTriggered,
              drivetrain::isGyroConnected,
              () -> zeroButtonTriggered,
              robotStates);
    } else { // REAL and SIM robots HERE
      switch (robotType) {
        case ROBOT_SIMBOT_REAL_CAMERAS:
        case ROBOT_SIMBOT:
          com.ctre.phoenix6.unmanaged.Unmanaged.setPhoenixDiagnosticsStartTime(0.0);

          drivetrain =
              new Drivetrain(
                  config,
                  new GyroIO.Fake(),
                  new GyroIO.Fake(),
                  config.getSwerveModuleObjects(),
                  () -> is_autonomous);

          if (robotType == RobotType.ROBOT_SIMBOT_REAL_CAMERAS) {
            // Sim Robot, Real Cameras
            vision =
                new Vision(
                    aprilTagLayout,
                    drivetrain::getHubPoseEstimatorPose,
                    drivetrain::getRotationGyroOnly,
                    drivetrain::addVisionEstimate,
                    config.getVisionModuleObjects());

            if (Constants.unusedCode) {
              visionGamepiece =
                  new VisionGamepiece(
                      new VisionGamepieceIOReal(), drivetrain::getHubPoseEstimatorPoseAtTimestamp);
            } else {
              visionGamepiece = null;
            }

          } else {
            VisionModuleConfiguration[] visionModules = {
              VisionModuleConfiguration.buildSim(
                  SimulatorRobotConfig.SHOOTER_SIDE_LEFT_CAMERA_NAME,
                  SimulatorRobotConfig.SHOOTER_SIDE_LEFT,
                  config,
                  drivetrain::getHubPoseEstimatorPose),
              VisionModuleConfiguration.buildSim(
                  SimulatorRobotConfig.SHOOTER_SIDE_RIGHT_CAMERA_NAME,
                  SimulatorRobotConfig.SHOOTER_SIDE_RIGHT,
                  config,
                  drivetrain::getHubPoseEstimatorPose),
            };

            vision =
                new Vision(
                    aprilTagLayout,
                    drivetrain::getHubPoseEstimatorPose,
                    drivetrain::getRotationGyroOnly,
                    drivetrain::addVisionEstimate,
                    visionModules);

            if (Constants.unusedCode) {
              visionGamepiece =
                  new VisionGamepiece(
                      new VisionGamepieceIOSim(config, drivetrain::getHubPoseEstimatorPose),
                      drivetrain::getHubPoseEstimatorPoseAtTimestamp);
            } else {
              visionGamepiece = null;
            }
          }

          intake = new Intake(new IntakeIOSim(), robotStates);
          climber = new Climber(new ClimberIOSim());
          led =
              new LED(
                  () -> brakeModeTriggered,
                  drivetrain::isGyroConnected,
                  () -> zeroButtonTriggered,
                  robotStates);
          break;

        case ROBOT_2023_RETIRED_ROBER:
          drivetrain =
              new Drivetrain(
                  config,
                  new GyroIOPigeon2(config, config.getGyroCANID()),
                  new GyroIOPigeon2(config, Constants.CanIDs.GYRO_2_CAN_ID),
                  config.getSwerveModuleObjects(),
                  () -> is_autonomous);

          vision =
              new Vision(
                  aprilTagLayout,
                  drivetrain::getHubPoseEstimatorPose,
                  drivetrain::getRotationGyroOnly,
                  drivetrain::addVisionEstimate,
                  config.getReplayVisionModules());
          intake = new Intake(new IntakeIO() {}, robotStates);
          climber = new Climber(new ClimberIO() {});

          if (Constants.unusedCode) {
            visionGamepiece =
                new VisionGamepiece(
                    new VisionGamepieceIO() {}, drivetrain::getHubPoseEstimatorPoseAtTimestamp);
          } else {
            visionGamepiece = null;
          }

          led =
              new LED(
                  () -> brakeModeTriggered,
                  drivetrain::isGyroConnected,
                  () -> zeroButtonTriggered,
                  robotStates);
          break;

        case ROBOT_2024_RETIRED_MAESTRO:
          drivetrain =
              new Drivetrain(
                  config,
                  new GyroIOPigeon2(config, config.getGyroCANID()),
                  new GyroIOPigeon2(config, Constants.CanIDs.GYRO_2_CAN_ID),
                  config.getSwerveModuleObjects(),
                  () -> is_autonomous);
          intake = new Intake(new IntakeIOReal(), robotStates);
          climber = new Climber(new ClimberIO() {});
          vision =
              new Vision(
                  aprilTagLayout,
                  drivetrain::getHubPoseEstimatorPose,
                  drivetrain::getRotationGyroOnly,
                  drivetrain::addVisionEstimate,
                  config.getVisionModuleObjects());

          if (Constants.unusedCode) {
            visionGamepiece =
                new VisionGamepiece(
                    new VisionGamepieceIOReal(), drivetrain::getHubPoseEstimatorPoseAtTimestamp);
          } else {
            visionGamepiece = null;
          }

          led =
              new LED(
                  () -> brakeModeTriggered,
                  drivetrain::isGyroConnected,
                  () -> zeroButtonTriggered,
                  robotStates);
          break;

        case ROBOT_2025_RETIRED_HOLO:
          try {
            Thread.sleep(1000);
          } catch (InterruptedException e) {
            System.out.println("sleep interrupted");
          }
          intake = new Intake(new IntakeIOReal(), robotStates);
          climber = new Climber(new ClimberIOReal());
          drivetrain =
              new Drivetrain(
                  config,
                  new GyroIOPigeon2(config, config.getGyroCANID()),
                  new GyroIO.Fake(),
                  config.getSwerveModuleObjects(),
                  () -> is_autonomous);
          vision =
              new Vision(
                  aprilTagLayout,
                  drivetrain::getHubPoseEstimatorPose,
                  drivetrain::getRotationGyroOnly,
                  drivetrain::addVisionEstimate,
                  config.getVisionModuleObjects());

          if (Constants.unusedCode) {
            visionGamepiece =
                new VisionGamepiece(
                    new VisionGamepieceIOReal(), drivetrain::getHubPoseEstimatorPoseAtTimestamp);
          } else {
            visionGamepiece = null;
          }

          led =
              new LED(
                  () -> brakeModeTriggered,
                  drivetrain::isGyroConnected,
                  () -> zeroButtonTriggered,
                  robotStates);

          break;

        case ROBOT_2026:
          try {
            Thread.sleep(1000);
          } catch (InterruptedException e) {
            System.out.println("sleep interrupted");
          }
          intake = new Intake(new IntakeIOReal(), robotStates);
          climber = new Climber(new ClimberIOReal());
          drivetrain =
              new Drivetrain(
                  config,
                  new GyroIOPigeon2(config, config.getGyroCANID()),
                  new GyroIO.Fake(),
                  config.getSwerveModuleObjects(),
                  () -> is_autonomous);
          vision =
              new Vision(
                  aprilTagLayout,
                  drivetrain::getHubPoseEstimatorPose,
                  drivetrain::getRotationGyroOnly,
                  drivetrain::addVisionEstimate,
                  config.getVisionModuleObjects());

          if (Constants.unusedCode) {
            visionGamepiece =
                new VisionGamepiece(
                    new VisionGamepieceIOReal(), drivetrain::getHubPoseEstimatorPoseAtTimestamp);
          } else {
            visionGamepiece = null;
          }

          led =
              new LED(
                  () -> brakeModeTriggered,
                  drivetrain::isGyroConnected,
                  () -> zeroButtonTriggered,
                  robotStates);
          break;

        default:
          drivetrain =
              new Drivetrain(
                  config,
                  new GyroIO.Fake(),
                  new GyroIO.Fake(),
                  config.getReplaySwerveModuleObjects(),
                  () -> is_autonomous);
          vision =
              new Vision(
                  aprilTagLayout,
                  drivetrain::getHubPoseEstimatorPose,
                  drivetrain::getRotationGyroOnly,
                  drivetrain::addVisionEstimate,
                  config.getReplayVisionModules());
          intake = new Intake(new IntakeIO() {}, robotStates);
          climber = new Climber(new ClimberIO() {});

          if (Constants.unusedCode) {
            visionGamepiece =
                new VisionGamepiece(
                    new VisionGamepieceIO() {}, drivetrain::getHubPoseEstimatorPoseAtTimestamp);
          } else {
            visionGamepiece = null;
          }

          led =
              new LED(
                  () -> brakeModeTriggered,
                  drivetrain::isGyroConnected,
                  () -> zeroButtonTriggered,
                  robotStates);
          break;
      }
    }

    drivetrainWrapper = new DrivetrainWrapper(drivetrain);

    flipAutoChooser.addDefaultOption("No", false);
    flipAutoChooser.addOption("Yes", true);

    for (int i = 1; i < customGamepieceCount; i++) { // Add custom pickup choosers
      LoggedDashboardChooser<PickupLocation> chooser =
          new LoggedDashboardChooser<>(i + " CustomPickup");
      for (PickupLocation side : PickupLocation.values()) {
        chooser.addOption(side.name(), side);
      }
      chooser.addDefaultOption(PickupLocation.IA.name(), PickupLocation.IA);
      coralStationPosChooser.add(chooser);
    }

    var subsystems = new AutosSubsystems(drivetrainWrapper, led, intake);

    robotStates.setSubsystems(
        intake, drivetrainWrapper, () -> driverController.getRightTrigger().getAsBoolean());

    autoManager =
        new AutosManager(
            subsystems,
            config,
            autoChooser,
            stringToAutoSupplierMap,
            () -> {
              var result = flipAutoChooser.get();
              if (result != null) {
                return result;
              }
              return false;
            },
            this::getCustomCoralStationLocations,
            robotStates);

    led.setPreloadSupplier(autoManager::preloadingComplete);

    drivetrain.setDefaultCommand(
        new DrivetrainDefaultTeleopDrive(
            drivetrainWrapper,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // ----------- DRIVER CONTROLS ------------

    if (Constants.unusedCode)
      driverController
          .registerTrigger(XboxControllerWrapper.Button.back, "Zero Robot")
          .onTrue(
              Commands.runOnce(
                  () -> {
                    Pose2d pose = drivetrain.getHubPoseEstimatorPose();
                    drivetrain.setPose(
                        new Pose2d(pose.getX(), pose.getY(), Constants.zeroRotation2d));
                  },
                  drivetrain));
    driverController
        .registerTrigger(XboxControllerWrapper.Button.start, "X Stance")
        .onTrue(Commands.runOnce(drivetrain::stopWithX));

    // ---------- OPERATOR CONTROLS -----------

    // Eject
    operatorController
        .registerTrigger(XboxControllerWrapper.Button.leftBumper, "Intake Eject")
        .onTrue(Commands.runOnce(() -> intake.setState(IntakeState.Eject)))
        .onFalse(Commands.runOnce(() -> intake.setState(IntakeState.Stow)));

    // Intake
    operatorController
        .registerTrigger(XboxControllerWrapper.Button.rightBumper, "Intake")
        .onTrue(Commands.runOnce(() -> intake.setState(IntakeState.IntakeCoral)))
        .onFalse(Commands.runOnce(() -> intake.setState(IntakeState.Stow)));

    // Intake positions
    operatorController
        .registerTrigger(XboxControllerWrapper.Button.a, "Intake Pivot In")
        .onTrue(Commands.runOnce(() -> intake.setState(IntakeState.Stow)));
    operatorController
        .registerTrigger(XboxControllerWrapper.Button.rightTrigger, "Intake Pivot Out")
        .onTrue(Commands.runOnce(() -> intake.setState(IntakeState.Down)));

    // Climber in
    operatorController
        .registerTrigger(XboxControllerWrapper.Button.b, "Climber In")
        .onTrue(
            Commands.runOnce(
                    () -> climber.setServoAngle(Constants.ClimberConstants.SERVO_UNLOCK_ANGLE))
                .andThen(Commands.waitSeconds(0.15))
                .andThen(
                    new ClimberSetAngle(climber, Constants.ClimberConstants.MIN_CLIMBER_ANGLE)));

    operatorController
        .registerTrigger(XboxControllerWrapper.Button.b, "Climber Out")
        .onTrue(new ClimberSetAngle(climber, Rotation2d.fromRotations(5.4)));

    operatorController
        .registerTrigger(XboxControllerWrapper.Button.rightStick, "Climber in")
        .onTrue(new ClimberSetAngle(climber, Rotation2d.fromRotations(0)));

    // ---------- ON-ROBOT CONTROLS ------------

    zeroSensorsButtonTrigger.onTrue(
        new RunsWhenDisabledInstantCommand(
            () -> {
              intake.resetPivotSensorToHomePosition();
              led.setRobotState(RobotState.ZERO_SUBSYSTEMS);
              climber.resetWinchSensorToHomePosition();
              zeroButtonTriggered = true;
            }));

    brakeModeButtonTrigger.onTrue(
        new ConditionalCommand(
            new RunsWhenDisabledInstantCommand(
                () -> {
                  boolean intakePivotSuccess = intake.setPivotNeutralMode(NeutralModeValue.Coast);
                  boolean climberWinchSuccess = climber.setWinchNeutralMode(NeutralModeValue.Coast);

                  brakeModeFailure = !intakePivotSuccess || !climberWinchSuccess;

                  brakeModeTriggered = false;
                  led.setRobotState(
                      brakeModeFailure ? RobotState.BRAKE_MODE_FAILED : RobotState.BRAKE_MODE_OFF);
                }),
            new RunsWhenDisabledInstantCommand(
                () -> {
                  boolean intakePivotSuccess = intake.setPivotNeutralMode(NeutralModeValue.Brake);
                  boolean climberWinchSuccess = climber.setWinchNeutralMode(NeutralModeValue.Brake);

                  brakeModeFailure = !intakePivotSuccess || !climberWinchSuccess;

                  brakeModeTriggered = true;
                  led.setRobotState(
                      brakeModeFailure ? RobotState.BRAKE_MODE_FAILED : RobotState.BRAKE_MODE_ON);
                }),
            () -> brakeModeTriggered));

    // ---------- ELASTIC CONTROLS ------------

    // Add Reset and Reboot buttons to SmartDashboard
    SmartDashboard.putData(
        "PV Restart SW 1_Shooter_Left",
        new RunsWhenDisabledInstantCommand(() -> Vision.restartPhotonVision("10.29.30.13")));

    SmartDashboard.putData(
        "PV REBOOT 1_Shooter_Left",
        new RunsWhenDisabledInstantCommand(() -> Vision.rebootPhotonVision("10.29.30.13")));

    SmartDashboard.putData(
        "PV Restart SW 2_Shooter_Right",
        new RunsWhenDisabledInstantCommand(() -> Vision.restartPhotonVision("10.29.30.14")));

    SmartDashboard.putData(
        "PV REBOOT 2_Shooter_Right",
        new RunsWhenDisabledInstantCommand(() -> Vision.rebootPhotonVision("10.29.30.14")));

    SmartDashboard.putData(
        "USE GYRO 1", new RunsWhenDisabledInstantCommand(() -> drivetrain.chooseWhichGyro(false)));
    SmartDashboard.putData(
        "USE GYRO 2", new RunsWhenDisabledInstantCommand(() -> drivetrain.chooseWhichGyro(true)));

    SmartDashboard.putData("Confirm Auto", new RunsWhenDisabledInstantCommand(led::confirmAuto));

    SmartDashboard.putData(
        "Swerve Coast", new RunsWhenDisabledInstantCommand(() -> drivetrain.setBrakeMode(false)));
    SmartDashboard.putData(
        "Swerve Brake", new RunsWhenDisabledInstantCommand(() -> drivetrain.setBrakeMode(true)));

    SmartDashboard.putData(
        "WheelRadiusCharacterisation",
        new WheelRadiusCharacterization(
            drivetrainWrapper, Constants.RobotMode.getRobot().config.get()));

    SmartDashboard.putData(
        "Zero Mech",
        new RunsWhenDisabledInstantCommand(
            () -> {
              intake.resetPivotSensorToHomePosition();
              led.setRobotState(RobotState.ZERO_SUBSYSTEMS);
            }));

    SmartDashboard.putData(
        "Zero Servo",
        new RunsWhenDisabledInstantCommand(
            () -> climber.setServoAngle(ClimberConstants.SERVO_LOCK_ANGLE)));

    SmartDashboard.putData(
        "Zero Robot Angle",
        Commands.runOnce(
            () -> {
              Pose2d pose = drivetrain.getHubPoseEstimatorPose();
              drivetrain.setPose(new Pose2d(pose.getX(), pose.getY(), Constants.zeroRotation2d));
            },
            drivetrain));
  }

  // TODO: Fix apriltagfinder code

  /*
  public static Pose2d findNearestAprilTag(Pose2d robotTranslation, Pose2d[] reefAprilTagPose) {
    Pose2d nearestAprilTag = null;
    int start;
    double minDistance = 10000000000.0;

    start = Constants.isRedAlliance() ? 0 : 6;

    for (int i = start; i < start + 6; i++) {
      Translation2d aprilTag = reefAprilTagPose[i].getTranslation();

      double distance = robotTranslation.getTranslation().getDistance(aprilTag);
      if (distance < minDistance) {
        minDistance = distance;
        nearestAprilTag = reefAprilTagPose[i];
      }
    }

    return nearestAprilTag;
  }


   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public LoggedDashboardChooser<String> getAutonomousChooser() {
    return autoChooser;
  }

  public boolean autoFlipped() {
    return flipAutoChooser.get();
  }

  public void setPose(Pose2d pose) {
    drivetrain.setPose(pose);
  }

  public void matchRawOdometryToPoseEstimatorValue() {
    drivetrain.setRawOdometryPose(drivetrain.getHubPoseEstimatorPose());
  }

  public void applyToDrivetrain() {
    drivetrainWrapper.apply();
  }

  public void enterDisabled() {
    robotStates.setRobotGoal(RobotGoal.None);

    resetSubsystems();
    vision.useMaxDistanceAwayFromExistingEstimate(false);
    vision.useGyroBasedFilteringForVision(false);

    if (!DriverStation.isFMSAttached()) {
      climber.setWinchVoltage(0);
      intake.setPivotVoltage(0);
      intake.setRollerPercentOut(0);
      robotStates.mechState = MechState.Idle;

      intake.setState(IntakeState.Idle);
    }

    is_teleop = false;
    is_autonomous = false;
  }

  public void enterAutonomous() {
    robotStates.setRobotGoal(RobotGoal.Auto);

    // TODO: add setting all motors into break mode if they aren't already
    vision.useMaxDistanceAwayFromExistingEstimate(true);
    vision.useGyroBasedFilteringForVision(true);

    if (visionGamepiece != null) {
      visionGamepiece.setPipelineIndex(0);
    }

    is_teleop = false;
    is_autonomous = true;
  }

  public void enterTeleop() {
    robotStates.setRobotGoal(RobotGoal.None);

    climber.setServoAngle(ClimberConstants.SERVO_UNLOCK_ANGLE);
    resetDrivetrainResetOverrides();
    vision.useMaxDistanceAwayFromExistingEstimate(true);
    vision.useGyroBasedFilteringForVision(true);

    led.setBaseRobotState(BaseRobotState.LEVEL_MODE);

    if (visionGamepiece != null) {
      visionGamepiece.setPipelineIndex(1);
    }

    is_teleop = true;
    is_autonomous = false;
  }

  public void resetDrivetrainResetOverrides() {
    drivetrainWrapper.resetVelocityOverride();
    drivetrainWrapper.resetRotationOverride();
  }

  public void resetSubsystems() {}

  public void updateRobotState() {
    robotStates.periodic();
  }

  public void getCustomScoringLocations() {}

  public List<PickupLocation> getCustomCoralStationLocations() {
    List<PickupLocation> coralStationLocations = new ArrayList<>();

    for (LoggedDashboardChooser<PickupLocation> location : coralStationPosChooser) {
      coralStationLocations.add(location.get());
    }

    return coralStationLocations;
  }

  public Command threadCommand() { // Used to set thread priority to main thread after a timed delay
    return Commands.sequence(
        Commands.waitSeconds(20),
        Commands.runOnce(() -> Threads.setCurrentThreadPriority(true, 10)).ignoringDisable(true));
  }
}
