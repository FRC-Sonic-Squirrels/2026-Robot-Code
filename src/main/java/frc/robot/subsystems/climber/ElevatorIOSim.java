package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import frc.lib.team2930.TalonFXSim;
import frc.robot.Constants;

// anything commented out is arm code that we are not using yet
public class ElevatorIOSim implements ElevatorIO {
  private TalonFXSim elevatorSim = new TalonFXSim(DCMotor.getKrakenX60Foc(1),
      Constants.ClimberConstants.INNER_CLIMBER_GEAR_RATIO, Constants.ClimberConstants.CLIMBER_MOI);

  private MotionMagicVoltage climberClosedLoopControl = new MotionMagicVoltage(0);

  public ElevatorIOSim() {
  }

  @Override
  public void updateElevatorInputs(Inputs inputs) {
    elevatorSim.update(Constants.kDefaultPeriod);

    inputs.position                 = elevatorSim.getPosition().in(Units.Degree);
    inputs.appliedVolts             = elevatorSim.getVoltage();
    inputs.velocityDegreesPerSecond = elevatorSim.getVelocity().in(Units.DegreesPerSecond);
  }

  @Override
  public void setElevatorClosedLoopPosition(double inches) {
    elevatorSim.setControl(climberClosedLoopControl.withPosition(inches));
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
