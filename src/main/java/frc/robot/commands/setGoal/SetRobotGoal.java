package frc.robot.commands.setGoal;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotStates;
import frc.robot.RobotStates.RobotGoal;

public class SetRobotGoal extends Command {
  private final RobotStates states;
  private final RobotGoal newGoal;
  private final boolean resetToNone;
  private final boolean overrideOthers;
  private boolean executed = false;

  public SetRobotGoal(
      RobotStates states, RobotGoal newGoal, boolean resetToNone, boolean overrideOthers) {
    this.states = states;
    this.newGoal = newGoal;
    this.resetToNone = resetToNone;
    this.overrideOthers = overrideOthers;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (overrideOthers) {
      states.setRobotGoal(newGoal);
      executed = true;
    } else {
      executed = states.setRobotGoalIf(RobotGoal.None, newGoal);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!executed) {
      boolean ex = states.setRobotGoalIf(RobotGoal.None, newGoal);
      if (ex) {
        executed = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (resetToNone && executed) {
      states.setRobotGoalIf(newGoal, RobotGoal.None);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
