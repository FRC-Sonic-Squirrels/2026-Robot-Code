package frc.robot.commands;

import frc.lib.team2930.StateMachine;
import frc.robot.RobotStates;
import frc.robot.autonomous.records.ChoreoTrajectoryWithName;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.BaseRobotState;
import frc.robot.subsystems.mechanism.Mechanism;
import frc.robot.subsystems.mechanism.arm.Arm;
import frc.robot.subsystems.mechanism.elevator.Elevator;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import java.util.Optional;
import java.util.function.Consumer;

public class ScoreCoral extends StateMachine {

  public ScoreCoral(
      DrivetrainWrapper wrapper,
      Mechanism mech,
      Elevator elevator,
      Arm arm,
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
