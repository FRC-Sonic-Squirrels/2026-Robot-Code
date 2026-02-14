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

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.*;
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
import frc.robot.RobotStates.IntakeGoal;
import frc.robot.RobotStates.RobotGoal;
import frc.robot.autonomous.AutosManager;
import frc.robot.autonomous.AutosManager.Auto;
import frc.robot.autonomous.AutosSubsystems;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.commands.drive.DrivetrainDefaultTeleopDrive;
import frc.robot.commands.drive.WheelRadiusCharacterization;
import frc.robot.commands.setGoal.SetRobotGoal;
import frc.robot.configs.SimulatorRobotConfig;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.BaseRobotState;
import frc.robot.subsystems.LED.RobotState;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOReal;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.Climber.ClimberGoal;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIOReal;
import frc.robot.subsystems.hopper.HopperIOSim;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOPigeon2;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionModuleConfiguration;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.HashMap;
import java.util.function.Supplier;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public String ROOT_TABLE = "RobotContainer";

  public static class Logs extends AutoRegisterLoggers {
    public LoggerEntry.EnumValue<RobotType> robotType;
    public LoggerEntry.EnumValue<Mode>      robotMode;
  }

  // Robot logging
  public final Logs logs = LoggerGroup.root.newAutoLogger(Logs.class, ROOT_TABLE);

  // Subsystems
  private final Drivetrain          drivetrain;
  private final DrivetrainWrapper   drivetrainWrapper;
  private final AprilTagFieldLayout aprilTagLayout;
  private final Vision              vision;
  private final Intake              rightIntake;
  private final Intake              leftIntake;
  private final LED                 led;
  private final Climber             climber;
  private final Hopper              hopper;
  private final Shooter             shooter;

  // Controllers
  private final XboxControllerWrapper driverController   = new XboxControllerWrapper(0);
  private final XboxControllerWrapper operatorController = new XboxControllerWrapper(1);

  // Auto
  private final LoggedDashboardChooser<String>  autoChooser     = new LoggedDashboardChooser<>("Auto Routine");
  private final LoggedDashboardChooser<Boolean> flipAutoChooser = new LoggedDashboardChooser<>("Flip Auto?");

  private final HashMap<String, Supplier<Auto>> stringToAutoSupplierMap = new HashMap<>();
  private final AutosManager                    autoManager;
  // Buttons

  public DigitalInput brakeModeButton   = new DigitalInput(0);
  public DigitalInput zeroSensorsButton = new DigitalInput(1);

  private final Trigger brakeModeButtonTrigger = new Trigger(
      () -> !brakeModeButton.get() && !DriverStation.isEnabled());

  private final Trigger zeroSensorsButtonTrigger = new Trigger(
      () -> !zeroSensorsButton.get() && !DriverStation.isEnabled());

  private boolean brakeModeTriggered  = true;
  private boolean zeroButtonTriggered = false;

  // Robot Modes
  @SuppressWarnings("unused")
  private boolean is_teleop;

  private boolean is_autonomous;

  private boolean brakeModeFailure = false;

  // Tunable numbers

  public static class Tunables extends AutoRegisterTunables {
    @TunableEntry(defaultValue = 13.75)
    public LoggedTunableNumber tunableX;

    @TunableEntry(defaultValue = 5.15)
    public LoggedTunableNumber tunableY;

    @TunableEntry(defaultValue = 0.0)
    public LoggedTunableNumber tunableAngle;

  }

  private final Tunables tunables = TunableNumberGroup.newAutoTunable(Tunables.class, "RobotContainer");

  private final RobotStates robotStates;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true); // Silence joystick warning

    RobotType robotType = Constants.RobotMode.getRobot();
    Mode      mode      = Constants.RobotMode.getMode();

    logs.robotType.info(robotType);
    logs.robotMode.info(mode);

    var config = robotType.config.get();
    aprilTagLayout = config.getAprilTagFieldLayout();

    String leftIntakeRoot  = "Intake/Left";
    String rightIntakeRoot = "Intake/Right";

    if (mode == Mode.REPLAY) {
      drivetrain = new Drivetrain(config, new GyroIO.Fake(), new GyroIO.Fake(), config.getReplaySwerveModuleObjects(),
          () -> is_autonomous);

      vision = new Vision(aprilTagLayout, drivetrain::getHubPoseEstimatorPose, drivetrain::getRotationGyroOnly,
          drivetrain::addVisionEstimate, config.getReplayVisionModules());

      leftIntake  = new Intake(leftIntakeRoot, new IntakeIO() {
                  });
      rightIntake = new Intake(rightIntakeRoot, new IntakeIO() {
                  });
      climber     = new Climber(new ClimberIO() {
                  });
      hopper      = new Hopper(new HopperIO() {
                  });
      shooter     = new Shooter(new ShooterIO() {
                  });

      led = new LED(() -> brakeModeTriggered, drivetrain::isGyroConnected, () -> zeroButtonTriggered);
    } else { // REAL and SIM robots HERE
      switch (robotType) {
        case ROBOT_SIMBOT_REAL_CAMERAS:
        case ROBOT_SIMBOT:
          com.ctre.phoenix6.unmanaged.Unmanaged.setPhoenixDiagnosticsStartTime(0.0);

          drivetrain = new Drivetrain(config, new GyroIO.Fake(), new GyroIO.Fake(), config.getSwerveModuleObjects(),
              () -> is_autonomous);

          if (robotType == RobotType.ROBOT_SIMBOT_REAL_CAMERAS) {
            // Sim Robot, Real Cameras
            vision = new Vision(aprilTagLayout, drivetrain::getHubPoseEstimatorPose, drivetrain::getRotationGyroOnly,
                drivetrain::addVisionEstimate, config.getVisionModuleObjects());
          } else {
            var                         config2       = (SimulatorRobotConfig) config;
            VisionModuleConfiguration[] visionModules = {
                VisionModuleConfiguration.buildSim(config2.SHOOTER_SIDE_LEFT_CAMERA_NAME, config2.SHOOTER_SIDE_LEFT,
                    config, drivetrain::getHubPoseEstimatorPose),
                VisionModuleConfiguration.buildSim(config2.SHOOTER_SIDE_RIGHT_CAMERA_NAME, config2.SHOOTER_SIDE_RIGHT,
                    config, drivetrain::getHubPoseEstimatorPose),
            };

            vision = new Vision(aprilTagLayout, drivetrain::getHubPoseEstimatorPose, drivetrain::getRotationGyroOnly,
                drivetrain::addVisionEstimate, visionModules);

            vision.logs.configTagAmount.info(config.getAprilTagFieldLayout().getTags().size());
          }

          leftIntake = new Intake(leftIntakeRoot, new IntakeIOSim());
          rightIntake = new Intake(rightIntakeRoot, new IntakeIOSim());
          climber = new Climber(new ClimberIOSim());
          hopper = new Hopper(new HopperIOSim());
          shooter = new Shooter(new ShooterIOSim());
          led = new LED(() -> brakeModeTriggered, drivetrain::isGyroConnected, () -> zeroButtonTriggered);
          break;

        case ROBOT_2025_RETIRED_HOLO:
          try {
            Thread.sleep(1000);
          } catch (InterruptedException e) {
            System.out.println("sleep interrupted");
          }
          leftIntake = new Intake(leftIntakeRoot, new IntakeIO() {
          });
          rightIntake = new Intake(rightIntakeRoot, new IntakeIO() {
          });
          climber = new Climber(new ClimberIOReal());
          drivetrain = new Drivetrain(config, new GyroIOPigeon2(config, config.getGyroCANID()), new GyroIO.Fake(),
              config.getSwerveModuleObjects(), () -> is_autonomous);
          hopper = new Hopper(new HopperIO() {
          });
          shooter = new Shooter(new ShooterIO() {
          });
          vision = new Vision(aprilTagLayout, drivetrain::getHubPoseEstimatorPose, drivetrain::getRotationGyroOnly,
              drivetrain::addVisionEstimate, config.getVisionModuleObjects());

          led = new LED(() -> brakeModeTriggered, drivetrain::isGyroConnected, () -> zeroButtonTriggered);

          break;

        case ROBOT_2026:
          try {
            Thread.sleep(1000);
          } catch (InterruptedException e) {
            System.out.println("sleep interrupted");
          }
          if (Constants.unusedCode) {
            IntakeIOConfig leftIntakeIOConfig  = new IntakeIOConfig(Constants.CanIDs.INTAKE_LEFT_PIVOT_CAN_ID,
                Constants.CanIDs.INTAKE_LEFT_ROLLER_CAN_ID, InvertedValue.Clockwise_Positive,
                InvertedValue.Clockwise_Positive);
            IntakeIOConfig rightIntakeIOConfig = new IntakeIOConfig(Constants.CanIDs.INTAKE_RIGHT_PIVOT_CAN_ID,
                Constants.CanIDs.INTAKE_RIGHT_ROLLER_CAN_ID, InvertedValue.Clockwise_Positive,
                InvertedValue.Clockwise_Positive);
            leftIntake  = new Intake(leftIntakeRoot, new IntakeIOReal(leftIntakeIOConfig));
            rightIntake = new Intake(rightIntakeRoot, new IntakeIOReal(rightIntakeIOConfig));
            climber     = new Climber(new ClimberIOReal());
            shooter     = new Shooter(new ShooterIOReal());
          } else {
            leftIntake  = new Intake(leftIntakeRoot, new IntakeIO() {
                        });
            rightIntake = new Intake(rightIntakeRoot, new IntakeIO() {
                        });
            climber     = new Climber(new ClimberIO() {
                        });
            shooter     = new Shooter(new ShooterIO() {
                        });
          }
          drivetrain = new Drivetrain(config, new GyroIOPigeon2(config, config.getGyroCANID()), new GyroIO.Fake(),
              config.getSwerveModuleObjects(), () -> is_autonomous);
          hopper = new Hopper(new HopperIOReal() {
          });
          vision = new Vision(aprilTagLayout, drivetrain::getHubPoseEstimatorPose, drivetrain::getRotationGyroOnly,
              drivetrain::addVisionEstimate, config.getVisionModuleObjects());

          led = new LED(() -> brakeModeTriggered, drivetrain::isGyroConnected, () -> zeroButtonTriggered);
          break;

        default:
          drivetrain = new Drivetrain(config, new GyroIO.Fake(), new GyroIO.Fake(),
              config.getReplaySwerveModuleObjects(), () -> is_autonomous);
          vision = new Vision(aprilTagLayout, drivetrain::getHubPoseEstimatorPose, drivetrain::getRotationGyroOnly,
              drivetrain::addVisionEstimate, config.getReplayVisionModules());
          leftIntake = new Intake(leftIntakeRoot, new IntakeIO() {
          });
          rightIntake = new Intake(rightIntakeRoot, new IntakeIO() {
          });
          climber = new Climber(new ClimberIO() {
          });
          hopper = new Hopper(new HopperIO() {
          });
          shooter = new Shooter(new ShooterIO() {
          });

          led = new LED(() -> brakeModeTriggered, drivetrain::isGyroConnected, () -> zeroButtonTriggered);
          break;
      }
    }

    drivetrainWrapper = new DrivetrainWrapper(drivetrain);

    flipAutoChooser.addDefaultOption("No", false);
    flipAutoChooser.addOption("Yes", true);

    var subsystems = new AutosSubsystems(drivetrainWrapper, led, leftIntake, rightIntake);

    robotStates = new RobotStates(leftIntake, rightIntake, shooter, drivetrainWrapper);

    autoManager = new AutosManager(subsystems, config, autoChooser, stringToAutoSupplierMap, () -> {
      var result = flipAutoChooser.get();
      if (result != null) {
        return result;
      }
      return false;
    }, robotStates);

    led.setPreloadSupplier(autoManager::preloadingComplete);

    drivetrain.setDefaultCommand(new DrivetrainDefaultTeleopDrive(drivetrainWrapper, () -> -driverController.getLeftY(),
        () -> -driverController.getLeftX(), () -> -driverController.getRightX()));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // ----------- DRIVER CONTROLS ------------

    if (Constants.unusedCode) {
      driverController.registerTrigger(XboxControllerWrapper.Button.back, "Zero Robot").onTrue(Commands.runOnce(() -> {
        Pose2d pose = drivetrain.getHubPoseEstimatorPose();
        drivetrain.setPose(new Pose2d(pose.getX(), pose.getY(), Constants.zeroRotation2d));
      }, drivetrain));
    }
    driverController.registerTrigger(XboxControllerWrapper.Button.start, "X Stance")
        .onTrue(Commands.runOnce(drivetrain::stopWithX));
    driverController.registerTrigger(XboxControllerWrapper.Button.a, "DriveToTag").whileTrue(Commands.repeatingSequence(
        new DriveToPose(drivetrainWrapper, false,
            aprilTagLayout.getTagPose(15).get().toPose2d()
                .transformBy(new Transform2d(Units.Inches.of(60).in(Units.Meters),
                    Units.Inches.of(-15).in(Units.Meters), Rotation2d.kZero)),
            () -> drivetrainWrapper.getHubPoseEstimatorPose(true)),
        new DriveToPose(drivetrainWrapper, false,
            aprilTagLayout.getTagPose(15).get().toPose2d()
                .transformBy(new Transform2d(Units.Inches.of(40).in(Units.Meters), Units.Inches.of(0).in(Units.Meters),
                    Rotation2d.kZero)),
            () -> drivetrainWrapper.getHubPoseEstimatorPose(true)),
        new DriveToPose(drivetrainWrapper, false,
            aprilTagLayout.getTagPose(15).get().toPose2d().transformBy(new Transform2d(
                Units.Inches.of(60).in(Units.Meters), Units.Inches.of(15).in(Units.Meters), Rotation2d.kZero)),
            () -> drivetrainWrapper.getHubPoseEstimatorPose(true))));
    driverController.registerTrigger(XboxControllerWrapper.Button.b, "DriveToTag2")
        .onTrue(new DriveToPose(drivetrainWrapper, false,
            aprilTagLayout.getTagPose(15).get().toPose2d().transformBy(new Transform2d(
                Units.Inches.of(60).in(Units.Meters), Units.Inches.of(15).in(Units.Meters), Rotation2d.kZero)),
            () -> drivetrainWrapper.getHubPoseEstimatorPose(true)));
    driverController.registerTrigger(XboxControllerWrapper.Button.leftTrigger, "Shoot")
        .onTrue(Commands.runOnce(() -> robotStates.setRobotGoal(RobotGoal.Score)))
        .onFalse(Commands.runOnce(() -> robotStates.setRobotGoal(RobotGoal.None)));
    driverController.registerTrigger(XboxControllerWrapper.Button.rightTrigger, "Pass")
        .onTrue(Commands.runOnce(() -> robotStates.setRobotGoal(RobotGoal.Pass)))
        .onFalse(Commands.runOnce(() -> robotStates.setRobotGoal(RobotGoal.None)));

    driverController.registerTrigger(XboxControllerWrapper.Button.leftBumper, "Score")
        .whileTrue(new SetRobotGoal(robotStates, RobotStates.RobotGoal.Score, true, false));

    // ---------- OPERATOR CONTROLS -----------

    // Eject
    operatorController.registerTrigger(XboxControllerWrapper.Button.leftBumper, "Left Intake Eject")
        .onTrue(Commands.runOnce(() -> leftIntake.setGoal(Intake.Goal.Eject)))
        .onFalse(Commands.runOnce(() -> leftIntake.setGoal(Intake.Goal.Stow)));
    operatorController.registerTrigger(XboxControllerWrapper.Button.rightBumper, "Right Intake Eject")
        .onTrue(Commands.runOnce(() -> rightIntake.setGoal(Intake.Goal.Eject)))
        .onFalse(Commands.runOnce(() -> rightIntake.setGoal(Intake.Goal.Stow)));

    // Intake
    operatorController.registerTrigger(XboxControllerWrapper.Button.leftTrigger, "Left Intake")
        .onTrue(Commands.runOnce(() -> robotStates.setIntakeGoal(IntakeGoal.LeftIntake)))
        .onFalse(Commands.runOnce(() -> robotStates.setIntakeGoal(IntakeGoal.Stow)));
    operatorController.registerTrigger(XboxControllerWrapper.Button.rightTrigger, "Right Intake")
        .onTrue(Commands.runOnce(() -> robotStates.setIntakeGoal(IntakeGoal.RightIntake)))
        .onFalse(Commands.runOnce(() -> robotStates.setIntakeGoal(IntakeGoal.Stow)));

    // Intake positions
    operatorController.registerTrigger(XboxControllerWrapper.Button.a, "Left Intake Pivot In")
        .onTrue(Commands.runOnce(() -> leftIntake.setGoal(Intake.Goal.Stow)));
    operatorController.registerTrigger(XboxControllerWrapper.Button.leftStick, "Left Intake Pivot Out")
        .onTrue(Commands.runOnce(() -> leftIntake.setGoal(Intake.Goal.Down)));
    operatorController.registerTrigger(XboxControllerWrapper.Button.x, "Right Intake Pivot In")
        .onTrue(Commands.runOnce(() -> rightIntake.setGoal(Intake.Goal.Stow)));
    operatorController.registerTrigger(XboxControllerWrapper.Button.rightStick, "Right Intake Pivot Out")
        .onTrue(Commands.runOnce(() -> rightIntake.setGoal(Intake.Goal.Down)));

    // ---------- ON-ROBOT CONTROLS ------------

    zeroSensorsButtonTrigger.onTrue(new RunsWhenDisabledInstantCommand(() -> {
      leftIntake.resetPivotSensorToHomePosition();
      rightIntake.resetPivotSensorToHomePosition();
      climber.setClimberGoal(ClimberGoal.Idle);
      led.setRobotState(RobotState.ZERO_SUBSYSTEMS);

      zeroButtonTriggered = true;
    }));

    brakeModeButtonTrigger.onTrue(new ConditionalCommand(new RunsWhenDisabledInstantCommand(() -> {
      boolean intakeLeftPivotSuccess  = leftIntake.setPivotNeutralMode(NeutralModeValue.Coast);
      boolean intakeRightPivotSuccess = rightIntake.setPivotNeutralMode(NeutralModeValue.Coast);
      boolean climberSuccess          = climber.setClimberNeutralMode(NeutralModeValue.Coast);

      brakeModeFailure = !intakeLeftPivotSuccess || !intakeRightPivotSuccess || !climberSuccess;

      brakeModeTriggered = false;
      led.setRobotState(brakeModeFailure ? RobotState.BRAKE_MODE_FAILED : RobotState.BRAKE_MODE_OFF);
    }), new RunsWhenDisabledInstantCommand(() -> {
      boolean intakeLeftPivotSuccess  = leftIntake.setPivotNeutralMode(NeutralModeValue.Brake);
      boolean intakeRightPivotSuccess = rightIntake.setPivotNeutralMode(NeutralModeValue.Brake);
      boolean climberSuccess          = climber.setClimberNeutralMode(NeutralModeValue.Brake);

      brakeModeFailure = !intakeLeftPivotSuccess || !intakeRightPivotSuccess || !climberSuccess;

      brakeModeTriggered = true;
      led.setRobotState(brakeModeFailure ? RobotState.BRAKE_MODE_FAILED : RobotState.BRAKE_MODE_ON);
    }), () -> brakeModeTriggered));

    // ---------- ELASTIC CONTROLS ------------

    // Add Reset and Reboot buttons to SmartDashboard
    SmartDashboard.putData("PV Restart SW 1_Shooter_Left",
        new RunsWhenDisabledInstantCommand(() -> Vision.restartPhotonVision("10.29.30.13")));

    SmartDashboard.putData("PV REBOOT 1_Shooter_Left",
        new RunsWhenDisabledInstantCommand(() -> Vision.rebootPhotonVision("10.29.30.13")));

    SmartDashboard.putData("PV Restart SW 2_Shooter_Right",
        new RunsWhenDisabledInstantCommand(() -> Vision.restartPhotonVision("10.29.30.14")));

    SmartDashboard.putData("PV REBOOT 2_Shooter_Right",
        new RunsWhenDisabledInstantCommand(() -> Vision.rebootPhotonVision("10.29.30.14")));

    SmartDashboard.putData("USE GYRO 1", new RunsWhenDisabledInstantCommand(() -> drivetrain.chooseWhichGyro(false)));
    SmartDashboard.putData("USE GYRO 2", new RunsWhenDisabledInstantCommand(() -> drivetrain.chooseWhichGyro(true)));

    SmartDashboard.putData("Confirm Auto", new RunsWhenDisabledInstantCommand(led::confirmAuto));

    SmartDashboard.putData("Swerve Coast", new RunsWhenDisabledInstantCommand(() -> drivetrain.setBrakeMode(false)));
    SmartDashboard.putData("Swerve Brake", new RunsWhenDisabledInstantCommand(() -> drivetrain.setBrakeMode(true)));

    SmartDashboard.putData("WheelRadiusCharacterisation",
        new WheelRadiusCharacterization(drivetrainWrapper, Constants.RobotMode.getRobot().config.get()));

    SmartDashboard.putData("Zero Mech", new RunsWhenDisabledInstantCommand(() -> {
      leftIntake.resetPivotSensorToHomePosition();
      leftIntake.resetPivotSensorToHomePosition();
      led.setRobotState(RobotState.ZERO_SUBSYSTEMS);
    }));

    SmartDashboard.putData("Zero Servo",
        new RunsWhenDisabledInstantCommand(() -> climber.setServoAngle(ClimberConstants.SERVO_LOCK_ANGLE)));

    SmartDashboard.putData("Zero Robot Angle", Commands.runOnce(() -> {
      Pose2d pose = drivetrain.getHubPoseEstimatorPose();
      drivetrain.setPose(new Pose2d(pose.getX(), pose.getY(), Constants.zeroRotation2d));
    }, drivetrain));
  }

  // TODO: Fix apriltagfinder code

  /*
   * public static Pose2d findNearestAprilTag(Pose2d robotTranslation, Pose2d[]
   * reefAprilTagPose) { Pose2d nearestAprilTag = null; int start; double
   * minDistance = 10000000000.0;
   *
   * start = Constants.isRedAlliance() ? 0 : 6;
   *
   * for (int i = start; i < start + 6; i++) { Translation2d aprilTag =
   * reefAprilTagPose[i].getTranslation();
   *
   * double distance = robotTranslation.getTranslation().getDistance(aprilTag); if
   * (distance < minDistance) { minDistance = distance; nearestAprilTag =
   * reefAprilTagPose[i]; } }
   *
   * return nearestAprilTag; }
   *
   *
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
      leftIntake.setPivotVoltage(0);
      rightIntake.setPivotVoltage(0);
      leftIntake.setRollerPercentOut(0);
      rightIntake.setRollerPercentOut(0);

      leftIntake.setGoal(Intake.Goal.Idle);
      rightIntake.setGoal(Intake.Goal.Idle);
    }

    is_teleop     = false;
    is_autonomous = false;
  }

  public void enterAutonomous() {
    robotStates.setRobotGoal(RobotGoal.Auto);

    // TODO: add setting all motors into break mode if they aren't already
    vision.useMaxDistanceAwayFromExistingEstimate(true);
    vision.useGyroBasedFilteringForVision(true);

    is_teleop     = false;
    is_autonomous = true;
  }

  public void enterTeleop() {
    robotStates.setRobotGoal(RobotGoal.None);

    climber.setServoAngle(ClimberConstants.SERVO_UNLOCK_ANGLE);
    resetDrivetrainResetOverrides();
    vision.useMaxDistanceAwayFromExistingEstimate(true);
    vision.useGyroBasedFilteringForVision(true);

    led.setBaseRobotState(BaseRobotState.LEVEL_MODE);

    is_teleop     = true;
    is_autonomous = false;
  }

  public void resetDrivetrainResetOverrides() {
    drivetrainWrapper.resetVelocityOverride();
    drivetrainWrapper.resetRotationOverride();
  }

  public void resetSubsystems() {
  }

  public void updateRobotState() {
    robotStates.periodic();
  }

  public Command threadCommand() { // Used to set thread priority to main thread after a timed delay
    return Commands.sequence(Commands.waitSeconds(20),
        Commands.runOnce(() -> Threads.setCurrentThreadPriority(true, 10)).ignoringDisable(true));
  }
}
