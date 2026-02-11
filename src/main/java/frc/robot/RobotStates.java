package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.team2930.AllianceFlipUtil;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.drive.DriveToPoseHelper;
import frc.robot.commands.drive.RotateToFaceHelper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.State;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import java.util.function.Supplier;

public class RobotStates {
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

  public enum TargetCoralPosition {
    Intake, EndEffector
  }

  public enum RobotGoal {
    None, Auto, Score, Pass, Climb
  }

  public enum IntakeGoal {
    Stow, LeftIntake, RightIntake
  }

  // Logging

  private LoggerGroup logGroup = LoggerGroup.build("RobotState");

  private LoggerEntry.Integer logPeriodicCounter = logGroup.buildInteger("Counter");

  private LoggerEntry.Bool logGamepieceInIntakeState = logGroup.buildBoolean("GamepieceInIntakeState");

  private LoggerEntry.EnumValue<EndEffectorDesiredAction> logEndEffectorDesiredAction = logGroup
      .buildEnum("EndEffectorDesiredAction");
  private LoggerEntry.EnumValue<MechState>                logMechState                = logGroup.buildEnum("MechState");

  private LoggerGroup      logSubsystemsInUse  = logGroup.subgroup("SubsystemsInUse");
  private LoggerEntry.Bool logMechInUse        = logSubsystemsInUse.buildBoolean("MechInUse");
  private LoggerEntry.Bool logEndEffectorInUse = logSubsystemsInUse.buildBoolean("EndEffectorInUse");

  private LoggerGroup      logSubsystemsInTargetStates = logGroup.subgroup("SubsystemsInTargetStates");
  private LoggerEntry.Bool logIntakeInTargetState      = logSubsystemsInTargetStates
      .buildBoolean("IntakeInTargetState");
  private LoggerEntry.Bool logMechInTargetState        = logSubsystemsInTargetStates.buildBoolean("MechInTargetState");
  private LoggerEntry.Text logGoal                     = logSubsystemsInTargetStates.buildString("Goal");
  private LoggerEntry.Text logWantedGoal               = logSubsystemsInTargetStates.buildString("WantedGoal");

  // States

  public EndEffectorDesiredAction endEffectorDesiredAction    = EndEffectorDesiredAction.Idle;
  public double                   endEffectorOverrideVelocity = Double.NaN;

  public MechState mechState = MechState.Idle;

  public State intakeState = State.Idle;

  public boolean fuelInIntake;

  public boolean highStowMode = true;

  public TargetCoralPosition targetCoralPosition = TargetCoralPosition.EndEffector;

  public boolean endEffectorInUse;
  public boolean mechanismInUse;
  public boolean intakeInUse;

  public boolean intakeInTargetState;
  public boolean mechInTargetState;

  public boolean coralExpectedInEndEffector = true;
  public boolean coralExpectedInIntake      = true;

  private RobotGoal goal       = RobotGoal.None;
  private RobotGoal wantedGoal = RobotGoal.None;

  private IntakeGoal intakeGoal       = IntakeGoal.Stow;
  private IntakeGoal wantedIntakeGoal = IntakeGoal.Stow;

  // Commands

  public RobotStates() {
  }

  private int periodicCounter;

  private Intake            leftIntake;
  private Intake            rightIntake;
  private DrivetrainWrapper drivetrain;
  private Shooter           shooter;

  private RotateToFaceHelper rtfHelper;
  private DriveToPoseHelper  dtpHelpter;
  private boolean            driveSlow  = false;
  private Pose2d             targetPose = null;
  private Supplier<Boolean>  intaking   = null;

  public void setSubsystems(Intake leftIntake, Intake rightIntake, Shooter shooter, DrivetrainWrapper drivetrain,
      Supplier<Boolean> intaking) {
    this.leftIntake  = leftIntake;
    this.rightIntake = rightIntake;
    this.drivetrain  = drivetrain;
    this.intaking    = intaking;
    this.shooter     = shooter;
    dtpHelpter       = new DriveToPoseHelper(this.drivetrain, () -> targetPose, () -> drivetrain.getRawOdometryPose(),
        () -> driveSlow);
    rtfHelper        = new RotateToFaceHelper(this.drivetrain, () -> targetPose.getTranslation(),
        () -> drivetrain.getRawOdometryPose());
  }

  public void changeEndEffectorIfNotAligning(EndEffectorDesiredAction action) {
    if (!endEffectorDesiredAction.alignmentActive) {
      endEffectorDesiredAction = action;
    }
  }

  private void handleGoalTransition() {
    if (wantedGoal == goal)
      return;
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
          intakeGoal = intakeGoal.Stow;
          return;
        }
        break;
    }
    intakeGoal = wantedIntakeGoal;
  }

  public void periodic() {
    periodicCounter = (periodicCounter + 1) % 100;
    logPeriodicCounter.info(periodicCounter);

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
        targetPose = AllianceFlipUtil
            .flipPoseForAlliance(new Pose2d(FieldConstants.BLUE_HUB_CENTER_POSE, Rotation2d.kZero));
        shoot();
        break;
      case Pass:
        targetPose = new Pose2d(AllianceFlipUtil.getClosestTranslation(
            new Translation2d[] { FieldConstants.RIGHT_PASS_AIM_POINT, FieldConstants.LEFT_PASS_AIM_POINT },
            drivetrain.getRawOdometryPose()), Rotation2d.kZero);
        shoot();
        break;
      case Climb:
        // TODO: set proper tower positon
        targetPose = new Pose2d(0, 0, Rotation2d.kZero);
        dtpHelpter.execute();
        break;
    }
    switch (intakeGoal) {
      case Stow:
        rightIntake.setState(intakeState.Stow);
        leftIntake.setState(intakeState.Stow);
      case LeftIntake:
        rightIntake.setState(intakeState.Stow);
        if (rightIntake.isStowed()) {
          leftIntake.setState(intakeState.Intake);
        }
      case RightIntake:
        leftIntake.setState(intakeState.Stow);
        if (leftIntake.isStowed()) {
          rightIntake.setState(intakeState.Intake);
        }
    }

    CommandScheduler.getInstance().setPeriod(endEffectorOverrideVelocity);

    logMechInUse.info(mechanismInUse);
    logEndEffectorInUse.info(endEffectorInUse);

    logIntakeInTargetState.info(intakeInTargetState);
    logMechInTargetState.info(mechInTargetState);

    if (mechState == MechState.Override) {
      mechanismInUse = true;
    }

    // Log states

    logEndEffectorDesiredAction.info(endEffectorDesiredAction);
    logMechState.info(mechState);
    logGoal.info(goal.name());
    logWantedGoal.info(wantedGoal.name());

    logGamepieceInIntakeState.info(fuelInIntake);

    mechanismInUse   = false;
    intakeInUse      = false;
    endEffectorInUse = false;
  }

  private void shoot() {
    rtfHelper.execute();
    // tell the shooter how far away we are from the target <- landon did this
    shooter.setFacingTarget(rtfHelper.withinTolerance());
    shooter.setGoal(Shooter.Goal.Shooting);
  }

  public void setMechState(MechState state) {
    mechState      = state;
    mechanismInUse = true;
  }

  public void setEndEffectorState(EndEffectorDesiredAction state) {
    endEffectorDesiredAction = state;
    endEffectorInUse         = true;
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
