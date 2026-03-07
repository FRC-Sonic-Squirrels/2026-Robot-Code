package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.lib.team2930.TalonFXElevatorSim;
import frc.robot.Constants;

public class ElevatorIOSim implements ElevatorIO {

  private VoltageOut         openLoopControl   = new VoltageOut(0);
  private MotionMagicVoltage closedLoopControl = new MotionMagicVoltage(0);

  private final TalonFXElevatorSim sim;

  public ElevatorIOSim(ElevatorIOConfig config) {
    this.sim = new TalonFXElevatorSim(new ElevatorSim(DCMotor.getKrakenX60Foc(1), config.gearRatio(),
        config.carriageMass(), config.pulleyDiameter() / 2.0, 0.0, config.maxHeight(), false, 0));
  }

  @Override
  public void updateInputs(Inputs inputs) {
    sim.update(Constants.kDefaultPeriod);
    inputs.appliedVolts            = sim.getVoltage();
    inputs.velocityInchesPerSecond = sim.getVelocity().in(Units.InchesPerSecond);
    inputs.heightInches            = sim.getPosition().in(Units.Inches);
  }

  @Override
  public void setVoltage(double volts) {
    sim.setControl(openLoopControl.withOutput(volts));
  }

  @Override
  public void setHeight(Distance height) {
    sim.setControl(closedLoopControl.withPosition(height.in(Units.Inch)));
  }

  @Override
  public void setSensorPosition(Distance position) {
    sim.setSensorPosition(position.in(Units.Inches));
  }

  @Override
  public void setClosedLoopConstants(double kP, double kD, double kG, double acceleration, double cruiseVelocity) {
    TalonFXConfiguration config       = new TalonFXConfiguration();
    Slot0Configs         slot0Configs = new Slot0Configs();

    slot0Configs.kP = kP;
    slot0Configs.kD = kD;
    slot0Configs.kG = kG;

    MotionMagicConfigs mmConfigs = new MotionMagicConfigs();
    mmConfigs.MotionMagicAcceleration   = acceleration;
    mmConfigs.MotionMagicCruiseVelocity = cruiseVelocity;

    config.Slot0       = slot0Configs;
    config.MotionMagic = mmConfigs;

    sim.setConfig(config);
  }
}
