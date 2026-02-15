package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.team2930.*;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.drive.DriveToPoseHelper;
import frc.robot.commands.drive.RotateToFaceHelper;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.DrivetrainWrapper;

public class RobotStates {
  private String ROOT_TABLE = "RobotStates";

  // End Effector states
  public enum EndEffectorDesiredAction {
    Idle(false), DepotIntake(false), GroundIntake(false), ShootFuel(true), PassCoralToEndEffector(false),
    PassToIntake(false);

    public final boolean alignmentActive;

    EndEffectorDesiredAction(boolean alignmentActive) {
      this.alignmentActive = alignmentActive;
    }
  }

  // Mechanism (Arm + Elevator) States

  public enum MechState {
    Idle, Override, StowPosition, ClimbPosition, AvoidIntake, Default, AutoPrep,
  }

  // Desired mech to hold coral

  public enum RobotGoal {
    None, Auto, Score, Pass, Climb
  }

  public enum IntakeGoal {
    Stow, LeftIntake, RightIntake
  }

  // Logging

  public static class SubsystemsInTargetStatesLogs extends AutoRegisterLoggers {
    public LoggerEntry.Text goal;
    public LoggerEntry.Text wantedGoal;
    public LoggerEntry.Text wantedIntakeGoal;
    public LoggerEntry.Text intakeGoal;

  }

  public static class SubsystemsInUseLogs extends AutoRegisterLoggers {
    public LoggerEntry.Bool intakeInUse;
    public LoggerEntry.Bool shooterInUse;
  }

  public static class Logs extends AutoRegisterLoggers {
    public LoggerEntry.Integer counter;

    public SubsystemsInUseLogs          subsystemsInUse;
    public SubsystemsInTargetStatesLogs subsystemsInTargetStates;
  }

  private final Logs logs = LoggerGroup.root.newAutoLogger(Logs.class, ROOT_TABLE);

  // States

  private RobotGoal goal       = RobotGoal.None;
  private RobotGoal wantedGoal = RobotGoal.None;

  private IntakeGoal intakeGoal       = IntakeGoal.Stow;
  private IntakeGoal wantedIntakeGoal = IntakeGoal.Stow;

  // Commands

  private final Intake             leftIntake;
  private final Intake             rightIntake;
  private final DrivetrainWrapper  drivetrain;
  private final Shooter            shooter;
  private final RotateToFaceHelper rtfHelper;
  private final DriveToPoseHelper  dtpHelpter;
  private final Hopper             hopper;

  private int     periodicCounter;
  private boolean driveSlow;
  private Pose2d  targetPose;

  public RobotStates(Intake leftIntake, Intake rightIntake, Shooter shooter, DrivetrainWrapper drivetrain,
      Hopper hopper) {
    this.leftIntake  = leftIntake;
    this.rightIntake = rightIntake;
    this.drivetrain  = drivetrain;
    this.shooter     = shooter;
    this.hopper      = hopper;
    dtpHelpter       = new DriveToPoseHelper(this.drivetrain, () -> targetPose, drivetrain::getRawOdometryPose,
        () -> driveSlow);
    rtfHelper        = new RotateToFaceHelper(this.drivetrain, () -> targetPose.getTranslation(),
        drivetrain::getRawOdometryPose);
  }

  private void handleGoalTransition() {
    if (wantedGoal == goal) {
      return;
    }
    if (DriverStation.isAutonomous() && wantedGoal != RobotGoal.Auto) {
      wantedGoal = RobotGoal.Auto;
      return;
    }
    switch (wantedGoal) {
      case None:
        drivetrain.resetVelocityOverride();
        drivetrain.resetRotationOverride();
        break;
      case Auto:
        break;
      case Score:
        break;
      case Pass:
        break;
      case Climb:
        break;
    }
    dtpHelpter.initialize();
    rtfHelper.initialize();
    goal = wantedGoal;
  }

  private void handleIntakeGoalTransition() {
    switch (wantedIntakeGoal) {
      case Stow:
        break;
      case LeftIntake:
      case RightIntake:
        if (goal == RobotGoal.Climb) {
          intakeGoal = IntakeGoal.Stow;
          return;
        }
        break;
    }
    intakeGoal = wantedIntakeGoal;
  }

  public void periodic() {
    periodicCounter = (periodicCounter + 1) % 100;
    logs.counter.info(periodicCounter);

    handleGoalTransition();
    handleIntakeGoalTransition();
    switch (goal) {
      case None:
        drivetrain.resetVelocityOverride();
        drivetrain.resetRotationOverride();
        shooter.setGoal(Shooter.Goal.Idle);
        break;
      case Auto:
        // TODO: run the auto state machene
        break;
      case Score:
        targetPose = FieldConstants.HUB_CENTER_POSE.getPose2d();
        shoot();
        shooter.setGoal(Shooter.Goal.Scoring);
        hopper.setGoal(Hopper.Goal.Shooting);
        break;
      case Pass:
        targetPose = new Pose2d(GeometryUtil.getClosestTranslation(new AllianceAwareTranslation2d[] {
            FieldConstants.RIGHT_PASS_AIM_POINT,
            FieldConstants.LEFT_PASS_AIM_POINT
        }, drivetrain.getRawOdometryPose()), Rotation2d.kZero);
        shoot();
        shooter.setGoal(Shooter.Goal.Passing);
        hopper.setGoal(Hopper.Goal.Shooting);
        break;
      case Climb:
        // TODO: set proper tower positon
        targetPose = new Pose2d(0, 0, Rotation2d.kZero);
        dtpHelpter.execute();
        break;
    }
    switch (intakeGoal) {
      case Stow:
        rightIntake.setGoal(Intake.Goal.Stow);
        leftIntake.setGoal(Intake.Goal.Stow);
        break;
      case LeftIntake:
        rightIntake.setGoal(Intake.Goal.Stow);
        if (rightIntake.isStowed()) {
          leftIntake.setGoal(Intake.Goal.Intake);
        }
        break;
      case RightIntake:
        leftIntake.setGoal(Intake.Goal.Stow);
        if (leftIntake.isStowed()) {
          rightIntake.setGoal(Intake.Goal.Intake);
        }
        break;
    }

    // Log states

    logs.subsystemsInTargetStates.goal.info(goal.name());
    logs.subsystemsInTargetStates.wantedGoal.info(wantedGoal.name());
    logs.subsystemsInTargetStates.wantedIntakeGoal.info(wantedIntakeGoal.name());
    logs.subsystemsInTargetStates.intakeGoal.info(intakeGoal.name());
  }

  private void shoot() {
    rtfHelper.execute();
    // tell the shooter how far away we are from the target <- landon did this
    shooter.setFacingTarget(rtfHelper.withinTolerance());
    shooter.setDistanceFromTarget(rtfHelper.getDistance());
  }

  public RobotGoal getRobotGoal() {
    return goal;
  }

  public void setRobotGoal(RobotGoal newGoal) {
    wantedGoal = newGoal;
  }

  public void setIntakeGoal(IntakeGoal newGoal) {
    wantedIntakeGoal = newGoal;
  }

  public boolean setRobotGoalIf(RobotGoal expetedGoal, RobotGoal newGoal) {
    if (getRobotGoal() == expetedGoal) {
      setRobotGoal(newGoal);
      return true;
    }
    return false;
  }
}
