package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.AllianceFlipUtil;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.RunStateMachineCommand;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.PassToEndEffector;
import frc.robot.commands.drive.DriveToPoseHelper;
import frc.robot.commands.drive.RotateToFaceHelper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import java.util.function.Supplier;

public class RobotStates {
  // End Effector states
  public enum EndEffectorDesiredAction {
    Idle(false),
    DepotIntake(false),
    GroundIntake(false),
    ShootFuel(true),
    PassCoralToEndEffector(false),
    PassToIntake(false);

    public final boolean alignmentActive;

    EndEffectorDesiredAction(boolean alignmentActive) {
      this.alignmentActive = alignmentActive;
    }
  }

  // Mechanism (Arm + Elevator) States

  public enum MechState {
    Idle,
    Override,
    StowPosition,
    ClimbPosition,
    AvoidIntake,
    Default,
    AutoPrep,
  }

  // Intake states

  public enum IntakeState {
    Idle,
    Override,
    IntakeFuel,
    PrepPassoff,
    PassoffEndEffector,
    PassoffIntake,
    ShootFuel,
    Stow,
    Climb,
    Down,
    Eject
  }

  // Desired mech to hold coral

  public enum TargetCoralPosition {
    Intake,
    EndEffector
  }

  public enum RobotGoal {
    None,
    Auto,
    Score,
    Pass,
    Climb
  }

  // Logging

  private LoggerGroup logGroup = LoggerGroup.build("RobotState");

  private LoggerEntry.Bool logGamepieceInIntakeState =
      logGroup.buildBoolean("GamepieceInIntakeState");

  private LoggerEntry.EnumValue<EndEffectorDesiredAction> logEndEffectorDesiredAction =
      logGroup.buildEnum("EndEffectorDesiredAction");
  private LoggerEntry.EnumValue<MechState> logMechState = logGroup.buildEnum("MechState");
  private LoggerEntry.EnumValue<IntakeState> logIntakeState = logGroup.buildEnum("IntakeState");

  private LoggerGroup logSubsystemsInUse = logGroup.subgroup("SubsystemsInUse");
  private LoggerEntry.Bool logMechInUse = logSubsystemsInUse.buildBoolean("MechInUse");
  private LoggerEntry.Bool logEndEffectorInUse =
      logSubsystemsInUse.buildBoolean("EndEffectorInUse");

  private LoggerGroup logSubsystemsInTargetStates = logGroup.subgroup("SubsystemsInTargetStates");
  private LoggerEntry.Bool logIntakeInTargetState =
      logSubsystemsInTargetStates.buildBoolean("IntakeInTargetState");
  private LoggerEntry.Bool logMechInTargetState =
      logSubsystemsInTargetStates.buildBoolean("MechInTargetState");
  private LoggerEntry.Text logGoal = logSubsystemsInTargetStates.buildString("Goal");
  private LoggerEntry.Text logWantedGoal = logSubsystemsInTargetStates.buildString("WantedGoal");

  // States

  public EndEffectorDesiredAction endEffectorDesiredAction = EndEffectorDesiredAction.Idle;
  public double endEffectorOverrideVelocity = Double.NaN;

  public MechState mechState = MechState.Idle;

  public IntakeState intakeState = IntakeState.Idle;

  public boolean fuelInIntake;

  public boolean highStowMode = true;

  public TargetCoralPosition targetCoralPosition = TargetCoralPosition.EndEffector;

  public boolean endEffectorInUse;
  public boolean mechanismInUse;
  public boolean intakeInUse;

  public boolean intakeInTargetState;
  public boolean mechInTargetState;

  public boolean coralExpectedInEndEffector = true;
  public boolean coralExpectedInIntake = true;

  private RobotGoal goal = RobotGoal.None;
  private RobotGoal wantedGoal = RobotGoal.None;

  // Triggers

  public Trigger triggerForCoralInRobot = new Trigger(() -> coralInRobot);
  public Trigger triggerForCoralInEndEffector = new Trigger(() -> coralInEndEffector).debounce(0.5);
  public Trigger triggerForCoralInIntake =
      new Trigger(() -> coralInIntake)
          .debounce(
              0.1); // debounce to allow coral to get fully in intake before ending intake command

  public Trigger instantTriggerForCoralInIntake = new Trigger(() -> coralInIntake);

  // Commands

  public RobotStates() {}

  private Intake intake;
  private DrivetrainWrapper drivetrain;

  private RotateToFaceHelper rtfHelper;
  private DriveToPoseHelper dtpHelpter;
  private boolean driveSlow = false;
  private Pose2d targetPose = null;
  private Supplier<Boolean> intaking = null;

  public void setSubsystems(
      Intake intake, DrivetrainWrapper drivetrain, Supplier<Boolean> intaking) {
    this.intake = intake;
    this.drivetrain = drivetrain;
    this.intaking = intaking;
    passCoralToEndEffector =
        new RunStateMachineCommand(() -> new PassToEndEffector(intake, this, true));
    passAlgaeToEndEffector =
        new RunStateMachineCommand(() -> new PassToEndEffector(intake, this, false));
    dtpHelpter =
        new DriveToPoseHelper(
            this.drivetrain,
            () -> targetPose,
            () -> drivetrain.getRawOdometryPose(),
            () -> driveSlow);
    rtfHelper =
        new RotateToFaceHelper(
            this.drivetrain,
            () -> targetPose.getTranslation(),
            () -> drivetrain.getRawOdometryPose());
  }

  public void changeEndEffectorIfNotAligning(EndEffectorDesiredAction action) {
    if (!endEffectorDesiredAction.alignmentActive) {
      endEffectorDesiredAction = action;
    }
  }

  private void handleGoalTransition() {
    if (wantedGoal == goal) return;
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
    intake.setState(IntakeState.Idle);
    dtpHelpter.initialize();
    rtfHelper.initialize();
    goal = wantedGoal;
  }

  public void periodic() {
    handleGoalTransition();
    switch (goal) {
      case None:
        drivetrain.resetVelocityOverride();
        drivetrain.resetRotationOverride();
        intakeIfWanted();
        break;
      case Auto:
        // TODO: run the auto state machene
        break;
      case Score:
        targetPose =
            AllianceFlipUtil.flipPoseForAlliance(
                new Pose2d(FieldConstants.BLUE_HUB_CENTER_POSE, Rotation2d.kZero));
        shoot();
        intakeIfWanted();
        break;
      case Pass:
        targetPose =
            new Pose2d(
                AllianceFlipUtil.getClosestTranslation(
                    new Translation2d[] {
                      FieldConstants.RIGHT_PASS_AIM_POINT, FieldConstants.LEFT_PASS_AIM_POINT
                    },
                    drivetrain.getRawOdometryPose()),
                Rotation2d.kZero);
        shoot();
        intakeIfWanted();
        break;
      case Climb:
        // TODO: set proper tower positon
        targetPose = new Pose2d(0, 0, Rotation2d.kZero);
        dtpHelpter.execute();
        break;
    }

    targetCoralPosition =
        scoringLevel == ScoringLevel.L1
            ? TargetCoralPosition.Intake
            : TargetCoralPosition
                .EndEffector; // If scoring in L1, target mech for coral is intake, else end
    // effector

    transferCoralToEndEffector =
        targetCoralPosition == TargetCoralPosition.EndEffector
            && !coralInEndEffector
            && coralInIntake;

    CommandScheduler.getInstance().setPeriod(endEffectorOverrideVelocity);

    transferAlgaeToEndEffector = false;
    // algaeInIntake && !algaeInEndEffector;

    transferCoralToIntake =
        targetCoralPosition == TargetCoralPosition.Intake && !coralInIntake && coralInEndEffector;

    logTransferCoralToEndEffector.info(transferCoralToEndEffector);
    logTransferCoralToIntake.info(transferCoralToIntake);

    logTransferAlgaeToEndEffector.info(transferAlgaeToEndEffector);
    logTransferAlgaeToIntake.info(transferAlgaeToIntake);

    logMechInUse.info(mechanismInUse);
    logEndEffectorInUse.info(endEffectorInUse);

    logIntakeInTargetState.info(intakeInTargetState);
    logMechInTargetState.info(mechInTargetState);

    if (mechState == MechState.Override) {
      mechanismInUse = true;
    }
    var scheduler = CommandScheduler.getInstance();

    if (!mechanismInUse && !endEffectorInUse) {

      // Passing coral between mechanisms
      if (transferCoralToEndEffector && !passCoralToEndEffector.isScheduled()) {
        scheduler.schedule(passCoralToEndEffector);
      }

      if (transferCoralToIntake && !passCoralToIntake.isScheduled()) {
        scheduler.schedule(passCoralToIntake);
      }

      // Passing algae between mechanisms
      if (transferAlgaeToEndEffector && !passAlgaeToEndEffector.isScheduled()) {
        scheduler.schedule(passAlgaeToEndEffector);
      }

      if (transferAlgaeToIntake && !passAlgaeToIntake.isScheduled()) {
        scheduler.schedule(passAlgaeToIntake);
      }
    } else {
      passCoralToEndEffector.cancel();
      passCoralToIntake.cancel();

      passAlgaeToEndEffector.cancel();
      passAlgaeToIntake.cancel();
    }

    // Updates states

    coralInRobot = coralInEndEffector || coralInIntake;

    algaeInRobot = algaeInEndEffector || algaeInIntake;

    // Log states

    logEndEffectorDesiredAction.info(endEffectorDesiredAction);
    logMechState.info(mechState);
    logGoal.info(goal.name());
    logWantedGoal.info(wantedGoal.name());

    logGamepieceInIntakeState.info(fuelInIntake);

    mechanismInUse = false;
    intakeInUse = false;
    endEffectorInUse = false;
  }

  private void intakeIfWanted() {
    if (intaking.get()) {
      intake.setState(IntakeState.IntakeCoral);
    } else {
      intake.setState(IntakeState.Idle);
    }
  }

  private void shoot() {
    rtfHelper.execute();
    // shooter.setAngle()hjmhjmh <- landon did this
    if (rtfHelper.withinTolerance() /*&& shooter.withinTolerance() */) {
      // pew pew
    }
  }

  public void setMechState(MechState state) {
    mechState = state;
    mechanismInUse = true;
  }

  public void setEndEffectorState(EndEffectorDesiredAction state) {
    endEffectorDesiredAction = state;
    endEffectorInUse = true;
  }

  public boolean coralInPassOff() {
    return passCoralToEndEffector.isScheduled() || passCoralToIntake.isScheduled();
  }

  public RobotGoal getRobotGoal() {
    return goal;
  }

  public void setRobotGoal(RobotGoal newGoal) {
    wantedGoal = newGoal;
  }

  // TODO: this should probrably be removed
  public void setIntakeState(Intake.IntakeState newState) {
    intake.setState(newState);
  }
}
