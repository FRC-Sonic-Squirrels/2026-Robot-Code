package frc.robot.subsystems.elevator;


import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import frc.lib.team2930.LoggerGroup;
import frc.robot.Constants;
import frc.robot.subsystems.BaseInputs;

public interface ElevatorIO {
  /** Contains all of the input data received from hardware. */
  class Inputs extends BaseInputs {
    public Distance position           = Distance.ofBaseUnits(0, Inches);
    public double     velocityInchesPerSecond = 0.0;
    public double     appliedVolts       = 0.0;
    public double     currentAmps        = 0.0;
    public double     tempCelsius        = 0.0;

    public Inputs(LoggerGroup logInputs) {
      super(logInputs);
    }

    // flywheel lead, flywheel follower, hood, kicker
    public double[] tempsCelsius = new double[4];
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(Inputs inputs) {
  }

  // Hood

  public default void setElevatorPosition(Distance pos) {
  }

  public default void setNeutralMode(NeutralModeValue value) {
  }

  public default void setElevatorClosedLoopConstants(Elevator.ElevatorTunables kicker) {
  }

  public default void setSensorPosition(Distance position) {
    
  }
}