package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TriggerWrapper {
  private final Trigger trigger;

  public TriggerWrapper(Trigger trigger) {
    this.trigger = trigger;
  }

  public TriggerWrapper onTrue(Command command) {
    return new TriggerWrapper(trigger.onTrue(command));
  }

  public TriggerWrapper onTrueOnce(Runnable runnable) {
    return new TriggerWrapper(trigger.onTrue(Commands.runOnce(runnable)));
  }

  public TriggerWrapper onFalse(Command command) {
    return new TriggerWrapper(trigger.onFalse(command));
  }

  public TriggerWrapper onFalseOnce(Runnable runnable) {
    return new TriggerWrapper(trigger.onFalse(Commands.runOnce(runnable)));
  }

  public TriggerWrapper whileTrue(Command command) {
    return new TriggerWrapper(trigger.whileTrue(command));
  }

  public TriggerWrapper react(TriggerOnce o) {
    return this.onTrueOnce(o::start).onFalseOnce(o::end);
  }
}
