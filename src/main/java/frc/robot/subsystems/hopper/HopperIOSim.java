package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import frc.lib.team2930.TalonFXSim;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants.RollerConstants;

public class HopperIOSim implements HopperIO {

  private TalonFXSim rollerMotor =
      new TalonFXSim(
          DCMotor.getKrakenX60Foc(1), RollerConstants.ROLLER_GEARING, RollerConstants.MOI);

  private VoltageOut rollerOpenLoopControl = new VoltageOut(0);
  private VelocityVoltage rollerClosedLoopControl = new VelocityVoltage(0);

  public HopperIOSim() {}

  @Override
  public void updateInputs(Inputs inputs) {
    rollerMotor.update(Constants.kDefaultPeriod);
    inputs.rollerAppliedVolts = rollerMotor.getVoltage();
    inputs.rollerVelocityRPM = rollerMotor.getVelocity().in(Units.RPM);
  }

  @Override
  public void setRollerVoltage(double volts) {
    rollerMotor.setControl(rollerOpenLoopControl.withOutput(volts));
  }

  @Override
  public void setRollerVelocity(double revPerMin) {
    rollerMotor.setControl(rollerClosedLoopControl.withVelocity(revPerMin));
  }

  @Override
  public void setRollerClosedLoopConstants(
      double kP, double kV, double kS, double maxProfiledAcceleration) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    Slot0Configs slot0Configs = new Slot0Configs();

    slot0Configs.kP = kP;
    slot0Configs.kV = kV;
    slot0Configs.kS = kS;

    config.Slot0 = slot0Configs;

    rollerMotor.setConfig(config);
  }
}
