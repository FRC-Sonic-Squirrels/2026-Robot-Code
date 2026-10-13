package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.lib.team2930.LoggerGroup;
import frc.robot.subsystems.BaseInputs;

public interface ShooterIO {
  /** Contains all of the input data received from hardware. */
  class Inputs extends BaseInputs {
    public double[] flywheelRPM          = new double[2];
    public double[] flywheelAppliedVolts = new double[2];
    public double[] flywheelCurrentAmps  = new double[2];

    public double kickerAppliedVolts = 0.0;
    public double kickerCurrentAmps  = 0.0;
    public double kickerVelocityRPM  = 0.0;
    public double kickerTargetRPM    = 0.0;

    public Inputs(LoggerGroup logInputs) {
      super(logInputs);
    }

    // flywheel lead, flywheel follower, hood, kicker
    public double[] tempsCelsius = new double[3];
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(Inputs inputs) {
  }

  // Flywheel

  public default void setFlywheelRPM(double revPerMin) {
  }

  public default void setFlywheelClosedLoopConstants(Shooter.FlywheelTunables tunables) {
  }

  // Kicker

  public default void setNeutralMode(NeutralModeValue value) {
  }

  public default void setKickerClosedLoopConstants(Shooter.KickerTunables kicker) {
  }

  public default void setKickerVelocity(double revPerMin) {
  }
}
