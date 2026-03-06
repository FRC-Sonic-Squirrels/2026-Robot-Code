package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.IntakeConstants.RollerConstants;
import frc.robot.Constants.MotorConstants.KrakenConstants;

public class HopperIOReal implements HopperIO {
  private final TalonFX rollerMotor;

  private final StatusSignal<Current>         rollerCurrent;
  private final StatusSignal<Temperature>     rollerDeviceTemp;
  private final StatusSignal<Voltage>         rollerAppliedVoltage;
  private final StatusSignal<AngularVelocity> rollerVelocity;

  private final BaseStatusSignal[] refreshSet;

  private final MotionMagicVelocityVoltage rollerClosedLoopControl = new MotionMagicVelocityVoltage(0)
      .withEnableFOC(true);

  public HopperIOReal(TalonFX roller) {
    rollerMotor = roller;

    // Motor config
    TalonFXConfiguration rollerConfig             = new TalonFXConfiguration();
    CurrentLimitsConfigs rollerCurrentLimitConfig = new CurrentLimitsConfigs();

    rollerCurrentLimitConfig.SupplyCurrentLimit       = RollerConstants.SUPPLY_CURRENT_LIMIT;
    rollerCurrentLimitConfig.SupplyCurrentLimitEnable = true;
    rollerCurrentLimitConfig.StatorCurrentLimit       = RollerConstants.STATOR_CURRENT_LIMIT;
    rollerCurrentLimitConfig.StatorCurrentLimitEnable = true;

    rollerConfig.CurrentLimits = rollerCurrentLimitConfig;

    rollerConfig.Feedback.SensorToMechanismRatio = RollerConstants.ROLLER_GEARING;
    rollerConfig.MotorOutput.Inverted            = InvertedValue.Clockwise_Positive;
    rollerConfig.MotorOutput.NeutralMode         = NeutralModeValue.Coast;

    rollerConfig.Voltage.SupplyVoltageTimeConstant = KrakenConstants.SUPPLY_VOLTAGE_TIME;

    rollerMotor.getConfigurator().apply(rollerConfig);

    // Status signals

    rollerCurrent        = rollerMotor.getStatorCurrent();
    rollerDeviceTemp     = rollerMotor.getDeviceTemp();
    rollerAppliedVoltage = rollerMotor.getMotorVoltage();
    rollerVelocity       = rollerMotor.getVelocity();

    // Update status signals

    BaseStatusSignal.setUpdateFrequencyForAll(50, rollerAppliedVoltage, rollerCurrent, rollerVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(1, rollerDeviceTemp);

    rollerMotor.optimizeBusUtilization();

    refreshSet = new BaseStatusSignal[] {
        rollerCurrent,
        rollerDeviceTemp,
        rollerAppliedVoltage,
        rollerVelocity,
    };
  }

  @Override
  public void updateInputs(Inputs inputs) {
    inputs.refreshAll(refreshSet);

    inputs.rollerCurrentAmps  = rollerCurrent.getValue().in(Units.Amps);
    inputs.rollerTempCelsius  = rollerDeviceTemp.getValue().in(Units.Celsius);
    inputs.rollerAppliedVolts = rollerAppliedVoltage.getValue().in(Units.Volts);
    inputs.rollerVelocityRPM  = rollerVelocity.getValue().in(Units.RPM);
  }

  @Override
  public void setRollerVelocity(double revPerMin) {
    rollerMotor.setControl(rollerClosedLoopControl.withVelocity(Units.RPM.of(revPerMin).in(Units.RotationsPerSecond)));
  }

  @Override
  public void setRollerClosedLoopConstants(double rKP, double rKV, double rKS, double targetAccelerationConfig) {
    Slot0Configs       pidConfig = new Slot0Configs();
    MotionMagicConfigs mmConfig  = new MotionMagicConfigs();

    var rollerConfig = rollerMotor.getConfigurator();

    rollerConfig.refresh(pidConfig);
    rollerConfig.refresh(mmConfig);

    pidConfig.kP = rKP;
    pidConfig.kV = rKV;
    pidConfig.kS = rKS;

    mmConfig.MotionMagicAcceleration = targetAccelerationConfig;

    rollerConfig.apply(pidConfig);
    rollerConfig.apply(mmConfig);
  }
}
