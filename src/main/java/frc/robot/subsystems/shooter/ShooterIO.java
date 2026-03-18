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
    public double[] tempsCelsius = new double[4];
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(Inputs inputs) {
  }

  // Hood

  default void setHoodPosition(Rotation2d rot) {
  }

  default void setHoodClosedLoopConstants(double kP, double kD, double kG, double maxProfiledVelocity,
      double maxProfiledAcceleration) {
  }

  default void resetHoodSensorPosition(Rotation2d angle) {
  }

  // Flywheel

  default void setFlywheelRPM(double revPerMin) {
  }

  default void setFlywheelClosedLoopConstants(Shooter.FlywheelTunables tunables) {
  }

  // Kicker

  default void setNeutralMode(NeutralModeValue value) {
  }

  default void setKickerClosedLoopConstants(Shooter.KickerTunables kicker) {
  }

  default void setKickerVelocity(double revPerMin) {
  }
}
