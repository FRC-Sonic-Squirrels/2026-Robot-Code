package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;

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

  // Logging

  private LoggerGroup logGroup = LoggerGroup.build("RobotState");

  private LoggerEntry.Bool logGamepieceInIntakeState =
      logGroup.buildBoolean("GamepieceInIntakeState");
  private LoggerEntry.Bool logCoralInIntakeState = logGroup.buildBoolean("CoralInIntakeState");

  private LoggerEntry.Bool logHighStowMode = logGroup.buildBoolean("HighStowMode");

  private LoggerEntry.EnumValue<EndEffectorDesiredAction> logEndEffectorDesiredAction =
      logGroup.buildEnum("EndEffectorDesiredAction");
  private LoggerEntry.EnumValue<MechState> logMechState = logGroup.buildEnum("MechState");
  private LoggerEntry.EnumValue<IntakeState> logIntakeState = logGroup.buildEnum("IntakeState");

  private LoggerGroup logTransferCoral = logGroup.subgroup("TransferCoral");
  private LoggerEntry.Bool logTransferCoralToEndEffector =
      logTransferCoral.buildBoolean("TransferToEndEffector");
  private LoggerEntry.Bool logTransferCoralToIntake =
      logTransferCoral.buildBoolean("TransferToIntake");
  private LoggerEntry.Bool logTransferAlgaeToEndEffector =
      logTransferCoral.buildBoolean("TransferToEndEffector");
  private LoggerEntry.Bool logTransferAlgaeToIntake =
      logTransferCoral.buildBoolean("TransferToIntake");

  private LoggerGroup logSubsystemsInUse = logGroup.subgroup("SubsystemsInUse");
  private LoggerEntry.Bool logIntakeInUse = logSubsystemsInUse.buildBoolean("IntakeInUse");
  private LoggerEntry.Bool logMechInUse = logSubsystemsInUse.buildBoolean("MechInUse");
  private LoggerEntry.Bool logEndEffectorInUse =
      logSubsystemsInUse.buildBoolean("EndEffectorInUse");

  private LoggerGroup logSubsystemsInTargetStates = logGroup.subgroup("SubsystemsInTargetStates");
  private LoggerEntry.Bool logIntakeInTargetState =
      logSubsystemsInTargetStates.buildBoolean("IntakeInTargetState");
  private LoggerEntry.Bool logMechInTargetState =
      logSubsystemsInTargetStates.buildBoolean("MechInTargetState");

  // States

  public EndEffectorDesiredAction endEffectorDesiredAction = EndEffectorDesiredAction.Idle;
  public double endEffectorOverrideVelocity = Double.NaN;

  public MechState mechState = MechState.Idle;

  public IntakeState intakeState = IntakeState.Idle;

  public boolean fuelInIntake;

  public boolean highStowMode = true;

  public TargetCoralPosition targetCoralPosition = TargetCoralPosition.EndEffector;

  private boolean transferCoralToEndEffector;
  private boolean transferCoralToIntake;

  private boolean transferAlgaeToEndEffector;
  private boolean transferAlgaeToIntake;

  public boolean endEffectorInUse;
  public boolean mechanismInUse;
  public boolean intakeInUse;

  public boolean intakeInTargetState;
  public boolean mechInTargetState;

  public boolean coralExpectedInEndEffector = true;
  public boolean coralExpectedInIntake = true;

  // Commands

  public RobotStates() {}

  public void changeEndEffectorIfNotAligning(EndEffectorDesiredAction action) {
    if (!endEffectorDesiredAction.alignmentActive) {
      endEffectorDesiredAction = action;
    }
  }

  public void periodic() {


    transferAlgaeToEndEffector = false;
    // algaeInIntake && !algaeInEndEffector;

    transferCoralToIntake =
        targetCoralPosition == TargetCoralPosition.Intake && !fuelInIntake;

    logTransferCoralToEndEffector.info(transferCoralToEndEffector);
    logTransferCoralToIntake.info(transferCoralToIntake);

    logTransferAlgaeToEndEffector.info(transferAlgaeToEndEffector);
    logTransferAlgaeToIntake.info(transferAlgaeToIntake);

    logIntakeInUse.info(intakeInUse);
    logMechInUse.info(mechanismInUse);
    logEndEffectorInUse.info(endEffectorInUse);

    logIntakeInTargetState.info(intakeInTargetState);
    logMechInTargetState.info(mechInTargetState);

    if (mechState == MechState.Override) {
      mechanismInUse = true;
    }

    if (intakeState == IntakeState.Override) {
      intakeInUse = true;
    }

    var scheduler = CommandScheduler.getInstance();

    if (!mechanismInUse && !endEffectorInUse && !intakeInUse) {

      // Log states

      logEndEffectorDesiredAction.info(endEffectorDesiredAction);
      logMechState.info(mechState);
      logIntakeState.info(intakeState);

      logGamepieceInIntakeState.info(fuelInIntake);
      logCoralInIntakeState.info(fuelInIntake);
      logHighStowMode.info(highStowMode);

      mechanismInUse = false;
      intakeInUse = false;
      endEffectorInUse = false;
    }
  }

  public void setMechState(MechState state) {
    mechState = state;
    mechanismInUse = true;
  }

  public void setIntakeState(IntakeState state) {
    intakeState = state;
    intakeInUse = true;
  }

  public void setEndEffectorState(EndEffectorDesiredAction state) {
    endEffectorDesiredAction = state;
    endEffectorInUse = true;
  }
}
