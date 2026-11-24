// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.AutoRegisterLoggers;
import frc.lib.team2930.AutoRegisterTunables;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team6328.LoggedTunableNumber;

public class Elevator2 extends SubsystemBase {

  public enum ElevatorGoal {
    HOME, L1, L2, L3
  }




static String ROOT_TABLE = "Elevator2";

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
public static class Logs extends AutoRegisterLoggers {
public LoggerEntry.Decimal appliedVolts;
public LoggerEntry.Decimal heightInches;
public LoggerEntry.Decimal targetHeightInches;

public LoggerEntry.EnumValue<ElevatorGoal> elevatorState; 
}

public final Logs logs = LoggerGroup.root.newAutoLogger(Logs.class, ROOT_TABLE); 

private ElevatorGoal goal;

  private ElevatorIO2 io;
private final ElevatorIO2.Inputs inputs = new ElevatorIO2.Inputs(logs.group);
  
private Distance elevatorTargetHeight;

  /** Creates a new Elevator. */
  public Elevator2(ElevatorIO2 io) {
    this.io = io;
    
  }

  @Override
  public void periodic() {
 io.updateInputs(inputs);
 logs.appliedVolts.info(inputs.appliedVolts);
 logs.appliedVolts.info(inputs.position.in(Inches));
    
  }

public double getTargetHeight(){
switch (goal) {
case HOME
return ElevatorConstants.HOME_HEIGHT.in(Inches);
case L1 
return ElevatorConstants.L1_HEIGHT



}
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
