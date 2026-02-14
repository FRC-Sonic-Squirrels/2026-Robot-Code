package frc.robot.subsystems.climber;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.team2930.LoggerGroup;
import frc.robot.subsystems.BaseInputs;

public interface ClimberIO {
  class Elevator extends BaseInputs {
    public double position;
    public double appliedVolts;
    public double currentAmps;
    public double tempCelsius;
    public double velocityDegreesPerSecond;

    public Elevator(LoggerGroup logInputs) {
      super(logInputs);
    }
  }

  class Inputs extends BaseInputs {
    private LoggerGroup logInputs;

    public Inputs(LoggerGroup logInputs) {
      super(logInputs);
      this.logInputs = logInputs;
    }

    public Elevator innerClimber = new Elevator(logInputs);
    public Elevator outerClimber = new Elevator(logInputs);
  }

  /** Updates the set of loggable inputs. */
  public default void updateClimberInputs(Inputs inputs) {
  }

  public default void resetClimberSensorPosition(Rotation2d angle) {
  }

  public default void setInnerClimberClosedLoopPosition(double inches) {
  }

  public default void setOuterClimberClosedLoopPosition(double inches) {
  }

  public default void setClimberServoAngle(Rotation2d angle) {
  }

  public default void setInnerClimberClosedLoopConstants(double kP, double kD, double kG) {
  }

  public default void setOuterClimberClosedLoopConstants(double kP, double kD, double kG) {
  }

  public default boolean setClimberNeutralMode(NeutralModeValue value) {
    return false;
  }
}
