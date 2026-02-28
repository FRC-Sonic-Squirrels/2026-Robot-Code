package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import frc.lib.team2930.TalonFXElevatorSim;
import frc.robot.Constants;

public class ElevatorIOSim implements ElevatorIO {
  private TalonFXElevatorSim elevatorSim;

  private MotionMagicVoltage climberClosedLoopControl = new MotionMagicVoltage(0);

  public ElevatorIOSim(TalonFXElevatorSim sim) {
    elevatorSim = sim;
  }

  @Override
  public void updateElevatorInputs(Inputs inputs) {
    elevatorSim.update(Constants.kDefaultPeriod);

    inputs.appliedVolts            = elevatorSim.getVoltage();
    inputs.velocityInchesPerSecond = elevatorSim.getVelocity().in(Units.InchesPerSecond);
    inputs.elevatorHeight          = elevatorSim.getPosition().in(Units.Inches);
  }

  @Override
  public void setElevatorClosedLoopPosition(Rotation2d position) {
    elevatorSim.setControl(climberClosedLoopControl.withPosition(position.getDegrees()));
  }

  @Override
  public void setElevatorClosedLoopConstants(double kP, double kD, double kG) {
    TalonFXConfiguration config       = new TalonFXConfiguration();
    Slot0Configs         slot0Configs = new Slot0Configs();

    slot0Configs.kP = kP;
    slot0Configs.kD = kD;
    slot0Configs.kG = kG;

    config.Slot0 = slot0Configs;

    elevatorSim.setConfig(config);
  }

  @Override
  public void resetElevatorSensorPosition(Rotation2d angle) {
    elevatorSim.setState(angle.getRadians(), 0.0);
  }
}
