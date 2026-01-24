package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotStates;
import frc.robot.subsystems.intake.Intake;

public class IntakeCoralGround extends Command {
  private final RobotStates states;

  /** Creates a new IntakeGround */
  public IntakeCoralGround(Intake intake, RobotStates states) {

    this.states = states;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }
}
