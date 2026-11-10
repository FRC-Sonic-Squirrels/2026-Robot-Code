// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.servohub.config.ServoChannelConfig.PulseRange;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.AutoRegisterTunables;
import frc.lib.team2930.TunableEntry;
import frc.lib.team6328.LoggedTunableNumber;

public class Elevator extends SubsystemBase {

  public enum ElevatorGoal  {
    HOME,
    L1,
    L2,
    L3
}


 public static class ElevatorTunables extends AutoRegisterTunables {
    public LoggedTunableNumber KP;
    public LoggedTunableNumber KI;
    public LoggedTunableNumber KD;
    public LoggedTunableNumber KS;
    public LoggedTunableNumber KV;
    public LoggedTunableNumber closedLoopMaxAccelerationConstraint;

    @TunableEntry(defaultValue = 0.0)
    public LoggedTunableNumber overrideRPM;

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
  
  private Distance elevatorTargetheight;


  /** Creates a new Elevator. */
  public Elevator() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public ElevatorGoal getGoal() {
      return goal;
  }
  public Distance getElevatorTargetheight(){
    return elevatorTargetheight; 
  }
  public void setGoal(ElevatorGoal goal) {
      this.goal = goal;
}
}
