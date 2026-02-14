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
public class ClimberIOSim implements ClimberIO {
  private TalonFXSim innerClimberSim = new TalonFXSim(DCMotor.getKrakenX60Foc(1),
      Constants.ClimberConstants.INNER_CLIMBER_GEAR_RATIO, Constants.ClimberConstants.CLIMBER_MOI);
  private TalonFXSim outerClimberSim = new TalonFXSim(DCMotor.getKrakenX60Foc(1),
      Constants.ClimberConstants.OUTER_CLIMBER_GEAR_RATIO, Constants.ClimberConstants.CLIMBER_MOI);

  private MotionMagicVoltage climberClosedLoopControl = new MotionMagicVoltage(0);

  public ClimberIOSim() {
  }

  @Override
  public void updateClimberInputs(Inputs inputs) {
    innerClimberSim.update(Constants.kDefaultPeriod);
    outerClimberSim.update(Constants.kDefaultPeriod);

    inputs.innerClimber.position                 = innerClimberSim.getPosition().in(Units.Degree);
    inputs.innerClimber.appliedVolts             = innerClimberSim.getVoltage();
    inputs.innerClimber.velocityDegreesPerSecond = innerClimberSim.getVelocity().in(Units.DegreesPerSecond);
    inputs.outerClimber.position                 = outerClimberSim.getPosition().in(Units.Degree);
    inputs.outerClimber.appliedVolts             = outerClimberSim.getVoltage();
    inputs.outerClimber.velocityDegreesPerSecond = outerClimberSim.getVelocity().in(Units.DegreesPerSecond);
  }

  @Override
  public void setInnerClimberClosedLoopPosition(double inches) {
    innerClimberSim.setControl(climberClosedLoopControl.withPosition(inches));
  }

  @Override
  public void setOuterClimberClosedLoopPosition(double inches) {
    outerClimberSim.setControl(climberClosedLoopControl.withPosition(inches));
  }

  @Override
  public void setInnerClimberClosedLoopConstants(double kP, double kD, double kG) {
    TalonFXConfiguration config       = new TalonFXConfiguration();
    Slot0Configs         slot0Configs = new Slot0Configs();

    slot0Configs.kP = kP;
    slot0Configs.kD = kD;
    slot0Configs.kG = kG;

    config.Slot0 = slot0Configs;

    innerClimberSim.setConfig(config);
  }

  @Override
  public void setOuterClimberClosedLoopConstants(double kP, double kD, double kG) {
    TalonFXConfiguration config       = new TalonFXConfiguration();
    Slot0Configs         slot0Configs = new Slot0Configs();

    slot0Configs.kP = kP;
    slot0Configs.kD = kD;
    slot0Configs.kG = kG;

    config.Slot0 = slot0Configs;

    outerClimberSim.setConfig(config);
  }

  @Override
  public void resetClimberSensorPosition(Rotation2d angle) {
    innerClimberSim.setState(angle.getRadians(), 0.0);
    outerClimberSim.setState(angle.getRadians(), 0.0);
  }
}
