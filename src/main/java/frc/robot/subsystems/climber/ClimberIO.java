package frc.robot.subsystems.climber;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.team2930.LoggerGroup;
import frc.robot.subsystems.BaseInputs;

public interface ClimberIO {
  class Inputs extends BaseInputs {
    public double innerClimberPosition;
    public double innerClimberAppliedVolts;
    public double innerClimberCurrentAmps;
    public double innerClimberTempCelsius;
    public double innerClimberVelocityDegreesPerSecond;
    public double outerClimberPosition;
    public double outerClimberAppliedVolts;
    public double outerClimberCurrentAmps;
    public double outerClimberTempCelsius;
    public double outerClimberVelocityDegreesPerSecond;

    public Inputs(LoggerGroup logInputs) {
      super(logInputs);
    }
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
