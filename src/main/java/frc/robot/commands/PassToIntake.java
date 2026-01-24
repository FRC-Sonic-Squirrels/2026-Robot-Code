package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.StateMachine;
import frc.robot.RobotStates;
import frc.robot.RobotStates.EndEffectorDesiredAction;
import frc.robot.RobotStates.MechState;
import frc.robot.subsystems.intake.Intake.IntakeState;

public class PassToIntake extends StateMachine {

  private final RobotStates states;
  private final Trigger endTrigger;

  public PassToIntake(RobotStates states) {
    super("PassToIntake");

    this.states = states;
    endTrigger = new Trigger(() -> states.coralInIntake).debounce(0.3);

    setInitialState(stateWithName("PrepToPassOff", () -> prepToPassOff()));
    setInterruptedState(stateWithName("End", () -> end()));
  }

  private StateHandler prepToPassOff() {
    states.mechanismInUse = true;
    states.intakeInUse = true;
    states.endEffectorInUse = true;
    states.mechState = MechState.PrepPassoffCoralPosition;
    states.setIntakeState(IntakeState.PrepPassoff);
    states.endEffectorDesiredAction = EndEffectorDesiredAction.PassToIntake;
    return states.intakeInTargetState && states.mechInTargetState
        ? stateWithName("PassOff", () -> passOff())
        : null;
  }

  private StateHandler passOff() {
    states.mechanismInUse = true;
    states.intakeInUse = true;
    states.endEffectorInUse = true;
    states.mechState = MechState.PassOffCoralPosition;
    states.setIntakeState(IntakeState.PassoffIntake);
    states.endEffectorDesiredAction = EndEffectorDesiredAction.PassToIntake;
    return endTrigger.getAsBoolean() ? () -> end() : null;
  }

  private StateHandler end() {
    states.mechState = MechState.StowPosition;
    states.setIntakeState(IntakeState.PrepPassoff);
    states.endEffectorDesiredAction = EndEffectorDesiredAction.AlignCoral;
    states.mechanismInUse = false;
    states.intakeInUse = false;
    states.endEffectorInUse = false;
    return setDone();
  }
}
