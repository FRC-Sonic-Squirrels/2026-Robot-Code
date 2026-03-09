package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.IntakeConstants.RollerConstants;
import frc.robot.Constants.MotorConstants.KrakenConstants;
import frc.robot.subsystems.TalonFXWrapper;

public class HopperIOReal implements HopperIO {
  private final TalonFXWrapper rollerMotor;

  private final StatusSignal<Current>         rollerCurrent;
  private final StatusSignal<Temperature>     rollerDeviceTemp;
  private final StatusSignal<Voltage>         rollerAppliedVoltage;
  private final StatusSignal<AngularVelocity> rollerVelocity;

  private final BaseStatusSignal[] refreshSet;

  private final MotionMagicVelocityVoltage rollerClosedLoopControl = new MotionMagicVelocityVoltage(0)
      .withEnableFOC(true);

  public HopperIOReal(TalonFXWrapper roller) {
    rollerMotor = roller;

    rollerMotor.talonFXConfiguration.change((rollerConfig) -> {
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
    });

    // Status signals

    rollerCurrent        = rollerMotor.io.getStatorCurrent();
    rollerDeviceTemp     = rollerMotor.io.getDeviceTemp();
    rollerAppliedVoltage = rollerMotor.io.getMotorVoltage();
    rollerVelocity       = rollerMotor.io.getVelocity();

    // Update status signals

    BaseStatusSignal.setUpdateFrequencyForAll(50, rollerAppliedVoltage, rollerCurrent, rollerVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(1, rollerDeviceTemp);

    rollerMotor.io.optimizeBusUtilization();

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
    if (revPerMin != 0) {
      rollerMotor.io
          .setControl(rollerClosedLoopControl.withVelocity(Units.RPM.of(revPerMin).in(Units.RotationsPerSecond)));
    } else {
      rollerMotor.io.setControl(new VoltageOut(0.0));
    }

  }

  @Override
  public void setRollerClosedLoopConstants(double rKP, double rKV, double rKS, double targetAccelerationConfig) {
    rollerMotor.slot0Configs.change((pidConfig) -> {
      pidConfig.kP = rKP;
      pidConfig.kV = rKV;
      pidConfig.kS = rKS;
    });

    rollerMotor.motionMagicConfigs.change((mmConfig) -> {
      mmConfig.MotionMagicAcceleration = targetAccelerationConfig;
    });
  }

  @Override
  public void setNeutralMode(NeutralModeValue value) {
    rollerMotor.setNeutralMode(value);
  }
}
