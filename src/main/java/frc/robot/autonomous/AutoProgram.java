package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoProgram {
  private final String  name;
  private final Command command;
  private final Pose2d  initialPose;

  public AutoProgram(Command command, Pose2d initialPose, String name) {
    this.command     = command;
    this.name        = name;
    this.initialPose = initialPose;
  }

  public String getName() {
    return name;
  }

  public Command getAutoCommand() {
    return command;
  }

  public Pose2d getInit() {
    return initialPose;
  }
}
