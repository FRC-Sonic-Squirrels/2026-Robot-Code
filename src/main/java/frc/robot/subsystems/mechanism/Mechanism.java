// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanism;

import frc.lib.team2930.ExecutionTiming;
import frc.robot.RobotStates;
import frc.robot.subsystems.mechanism.arm.Arm;
import frc.robot.subsystems.mechanism.elevator.Elevator;

/** Add your docs here. */
public class Mechanism {
  private static final String ROOT_TABLE = "Mechanism";

  // Execution timing
  private static final ExecutionTiming timing = new ExecutionTiming(ROOT_TABLE);

  // Arm and Elevator objects

  private final Elevator elevator;
  private final Arm arm;

  // Logging

  // States

  private boolean elevatorInPosition = false;
  private boolean armInPosition = false;
  private final RobotStates states;

  public Mechanism(Elevator elevator, Arm arm, RobotStates states) {
    this.elevator = elevator;
    this.arm = arm;
    this.states = states;
  }

  public void periodic() {
    try (var ignored = timing.start()) {
      // Enum based state machine
      switch (states.mechState) {
        case Idle:
          elevator.setPercentOut(0);
          arm.setPercentOut(0);
          break;
        case Override:
          break;
        default:
          break;
      }
      states.mechInTargetState = mechInPosition();
    }
  }

  public boolean mechInPosition() {
    return elevatorInPosition && armInPosition;
  }

  public Elevator getElevator() {
    return elevator;
  }

  public Arm getArm() {
    return arm;
  }

  /**
   * @param mech1
   * @param mech2
   * @return whether direct mech motion would not cause collision Sends mechanism to target position
   *     with little smart routing
   * @param position
   * @param elevatorAccel
   * @param armAccel
   */
}
