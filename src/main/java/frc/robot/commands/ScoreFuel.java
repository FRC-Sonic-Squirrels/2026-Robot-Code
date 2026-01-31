package frc.robot.commands;

import frc.lib.team2930.StateMachine;
import frc.robot.RobotStates;
import frc.robot.autonomous.records.ChoreoTrajectoryWithName;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.BaseRobotState;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import java.util.Optional;
import java.util.function.Consumer;

public class ScoreFuel extends StateMachine {

  public ScoreFuel(
      DrivetrainWrapper wrapper,
      LED led,
      Consumer<Double> rumble,
      RobotConfig config,
      boolean driverConfirmation,
      Optional<ChoreoTrajectoryWithName> optionalPresetPath,
      RobotStates robotStates) {
    super("ScoreCoral");

    led.setBaseRobotState(BaseRobotState.SCORING_ALIGNMENT);
  }
}
