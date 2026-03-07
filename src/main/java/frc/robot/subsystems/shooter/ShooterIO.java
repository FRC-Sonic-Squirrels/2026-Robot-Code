package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.team2930.LoggerGroup;
import frc.robot.Constants;
import frc.robot.subsystems.BaseInputs;

public interface ShooterIO {
  /** Contains all of the input data received from hardware. */
  class Inputs extends BaseInputs {
    public Rotation2d hoodPosition           = Constants.zeroRotation2d;
    public double     hoodVelocityRadsPerSec = 0.0;
    public double     hoodAppliedVolts       = 0.0;
    public double     hoodCurrentAmps        = 0.0;

    public double[] flywheelRPM          = new double[2];
    public double[] flywheelAppliedVolts = new double[2];
    public double[] flywheelCurrentAmps  = new double[2];

    public double kickerAppliedVolts = 0.0;
    public double kickerCurrentAmps  = 0.0;
    public double kickerVelocityRPM  = 0.0;
    public double kickerTargetRPM    = 0.0;

    public boolean fuelInUpperKicker = false;

    public Inputs(LoggerGroup logInputs) {
      super(logInputs);
    }

    // flywheel lead, flywheel follower, hood, kicker
    public double[] tempsCelcius = new double[4];
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(Inputs inputs) {
  }

  // Hood

  public default void setHoodPosition(Rotation2d rot) {
  }

  public default void setHoodClosedLoopConstants(double kP, double kD, double kG, double maxProfiledVelocity,
      double maxProfiledAcceleration) {
  }

  public default void resetHoodSensorPosition(Rotation2d angle) {
  }

  // Flywheel

  public default void setFlywheelRPM(double revPerMin) {
  }

  public default void setFlywheelClosedLoopConstants(double kP, double kV, double kS, double maxProfiledAcceleration) {
  }

  // Kicker

  public default void setNeutralMode(NeutralModeValue value) {
  }

  public default void setKickerClosedLoopConstants(double kP, double kV, double kS, double maxProfiledAcceleration) {
  }

  public default void setKickerVelocity(double revPerMin) {
  }
}
