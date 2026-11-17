// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.AutoRegisterTunables;
import frc.lib.team6328.LoggedTunableNumber;

public class Elevator2 extends SubsystemBase {

  public enum ElevatorGoal {
    HOME, L1, L2, L3
  }

  public static class ElevatorTunables extends AutoRegisterTunables {
    public LoggedTunableNumber KP;
    public LoggedTunableNumber KI;
    public LoggedTunableNumber KD;
    public LoggedTunableNumber KS;
    public LoggedTunableNumber KV;
    public LoggedTunableNumber closedLoopMaxAccelerationConstraint;

    public boolean hasChanged() {
      var hasChanged = false;
      hasChanged |= KP.hasChanged();
      hasChanged |= KI.hasChanged();
      hasChanged |= KD.hasChanged();
      hasChanged |= KV.hasChanged();
      hasChanged |= KS.hasChanged();
      hasChanged |= closedLoopMaxAccelerationConstraint.hasChanged();
      return hasChanged;
    }
  }

  private ElevatorGoal goal;

  private ElevatorIO2 io;

  private Distance elevatorTargetHeight;

  /** Creates a new Elevator. */
  public Elevator2(ElevatorIO2 io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public ElevatorGoal getGoal() {
    return goal;
  }

  public Distance getElevatorTargetHeight() {
    return elevatorTargetHeight;
  }

  public void setGoal(ElevatorGoal goal) {
    this.goal = goal;
  }
}
