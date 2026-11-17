package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.lib.team2930.TalonFXElevatorSim;

import frc.robot.Constants;

public class ElevatorIOSim2 implements ElevatorIO2 {

  private final TalonFXElevatorSim sim = new TalonFXElevatorSim(
      new ElevatorSim(DCMotor.getKrakenX60Foc(1), Constants.ElevatorConstants.GEAR_RATIO,
          Constants.ElevatorConstants.CARRIAGE_MASS, Constants.ElevatorConstants.PULLEY_DIAMETER.in(Units.Meters) / 2.0,
          0.0, Constants.ElevatorConstants.MAX_HEIGHT.in(Units.Meter), false, 0));

  private MotionMagicVoltage closedLoopControl = new MotionMagicVoltage(0);

  public void updateInputs(Inputs inputs) {
  }

  public void setElevatorPosition(Distance dist) {
  }

  public void setNeutralMode(NeutralModeValue value) {
  }

  public void setElevatorClosedLoopConstants(Elevator2.ElevatorTunables kicker) {
  }

  public void setSensorPosition(Distance position) {
  }
}
