package frc.robot.commands.setGoal;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotStates;
import frc.robot.RobotStates.IntakeGoal;

public class SetIntakingGoal extends Command {
  private final RobotStates states;
  private final IntakeGoal  newGoal;
  private final boolean     resetToStow;
  private final boolean     overrideOthers;
  private boolean           executed = false;

  public SetIntakingGoal(RobotStates states, IntakeGoal newGoal, boolean resetToStow, boolean overrideOthers) {
    this.states         = states;
    this.newGoal        = newGoal;
    this.resetToStow    = resetToStow;
    this.overrideOthers = overrideOthers;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (overrideOthers) {
      states.setIntakeGoal(newGoal);
      executed = true;
    } else {
      executed = states.setIntakeGoalIf(IntakeGoal.Stow, newGoal);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!executed) {
      boolean ex = states.setIntakeGoalIf(IntakeGoal.Stow, newGoal);
      if (ex) {
        executed = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (resetToStow && executed) {
      states.setIntakeGoalIf(newGoal, IntakeGoal.Stow);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
