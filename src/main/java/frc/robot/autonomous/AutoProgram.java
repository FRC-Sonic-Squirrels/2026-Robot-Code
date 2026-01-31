package frc.robot.autonomous;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoProgram {
  private final String                name;
  private final Pair<Command, Pose2d> autoPlan;

  public AutoProgram(Pair<Command, Pose2d> autoPlan, String name) {
    this.autoPlan = autoPlan;
    this.name     = name;
  }

  public String getName() {
    return name;
  }

  public Command getAutoCommand() {
    return autoPlan.getFirst();
  }

  public Pose2d getInit() {
    return autoPlan.getSecond();
  }
}
