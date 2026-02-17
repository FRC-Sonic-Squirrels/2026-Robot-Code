package frc.robot.subsystems.climber;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.team2930.LoggerGroup;
import frc.robot.subsystems.BaseInputs;

public interface ElevatorIO {

  class Inputs extends BaseInputs {
    public double position;
    public double appliedVolts;
    public double currentAmps;
    public double tempCelsius;
    public double velocityDegreesPerSecond;

    public Inputs(LoggerGroup logInputs) {
      super(logInputs);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateElevatorInputs(Inputs inputs) {
  }

  public default void resetElevatorSensorPosition(Rotation2d angle) {
  }

  public default void setElevatorClosedLoopPosition(double inches) {
  }

  public default void setElevatorClosedLoopConstants(double kP, double kD, double kG) {
  }

  public default boolean setElevatorNeutralMode(NeutralModeValue value) {
    return false;
  }
}
