package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public interface ShooterIO {
  /** Contains all of the input data received from hardware. */
  public static class Inputs {
    public Rotation2d hoodPosition = Constants.zeroRotation2d;
    public double hoodVelocityRadsPerSec = 0.0;
    public double hoodAppliedVolts = 0.0;
    public double hoodCurrentAmps = 0.0;

    public double[] flywheelRPM = new double[2];
    public double[] flywheelAppliedVolts = new double[2];
    public double[] flywheelCurrentAmps = new double[2];

    public double kickerAppliedVolts = 0.0;
    public double kickerCurrentAmps = 0.0;
    public double kickerVelocityRPM = 0.0;

    //  flywheel lead, flywheel follower, hood, kicker
    public double[] tempsCelcius = new double[4];
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(Inputs inputs) {}

  // HOOD
  public default void setHoodPosition(Rotation2d rot) {}

  public default void setHoodVoltage(double volts) {}

  public default void setHoodClosedLoopConstants(
      double kP,
      double kD,
      double kG,
      double maxProfiledVelocity,
      double maxProfiledAcceleration) {}

  // Flywheel
  public default void setFlywheelVoltage(double volts) {}

  public default void setFlywheelRPM(double topRollerRPM, double bottomRollerRPM) {}

  public default void setFlywheelClosedLoopConstants(
      double kP, double kV, double kS, double maxProfiledAcceleration) {}

  // KICKER
  public default void setKickerVoltage(double volts) {}

  public default boolean setNeutralMode(NeutralModeValue value) {
    return false;
  }

  public default void setKickerClosedLoopConstants(
      double kP, double kV, double kS, double maxProfiledAcceleration) {}

  public default void setKickerVelocity(double revPerMin) {}
}
