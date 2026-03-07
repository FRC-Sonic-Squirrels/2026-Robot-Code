package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.lib.team2930.LoggerGroup;
import frc.robot.subsystems.BaseInputs;

public interface HopperIO {

  class Inputs extends BaseInputs {
    public double rollerVelocityRPM;
    public double rollerCurrentAmps;
    public double rollerTempCelsius;
    public double rollerAppliedVolts;

    public Inputs(LoggerGroup logInputs) {
      super(logInputs);
    }
  }

  public default void updateInputs(Inputs inputs) {
  }

  public default void setNeutralMode(NeutralModeValue value) {
  }

  public default void setRollerVelocity(double revPerMin) {
  }

  public default void setRollerClosedLoopConstants(double kP, double kV, double kS, double targetAccelerationConfig) {
  }
}
