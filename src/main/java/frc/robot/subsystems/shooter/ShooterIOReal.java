package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.MotorConstants.KrakenConstants;

public class ShooterIOReal implements ShooterIO {
    private TalonFX flywheelLeadMotor = new TalonFX(Constants.CanIDs.SHOOTER_FLYWHEEL_LEAD_CAN_ID);
    private TalonFX flywheelFollowMotor = new TalonFX(Constants.CanIDs.SHOOTER_FLYWHEEL_FOLLOW_CAN_ID);

    private final MotionMagicVoltage closedLoopControl =
        new MotionMagicVoltage(0.0).withEnableFOC(true);
    private final VoltageOut openLoopControl = new VoltageOut(0.0).withEnableFOC(true);

    private StatusSignal<Angle> rotorPosition;
    private StatusSignal<AngularVelocity> rotorVelocity;
    private StatusSignal<Voltage> appliedVoltage;
    private StatusSignal<Current> current;
    private StatusSignal<Temperature> temp;

    private final BaseStatusSignal[] refreshSet;

    public ShooterIOReal() {

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimit = Constants.ShooterConstants.SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = Constants.ShooterConstants.STATOR_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.Voltage.SupplyVoltageTimeConstant = KrakenConstants.SUPPLY_VOLTAGE_TIME;

        flywheelLeadMotor.getConfigurator().apply(config);
        flywheelFollowMotor.getConfigurator().apply(config);
        
        // Status signals

        rotorPosition = flywheelLeadMotor.getRotorPosition();
        rotorVelocity = flywheelLeadMotor.getRotorVelocity();
        appliedVoltage = flywheelLeadMotor.getMotorVoltage();
        current = flywheelLeadMotor.getStatorCurrent();
        temp = flywheelLeadMotor.getDeviceTemp();

        // Update status signals

        BaseStatusSignal.setUpdateFrequencyForAll(100, rotorPosition, rotorVelocity);
        BaseStatusSignal.setUpdateFrequencyForAll(50, appliedVoltage, current);
        BaseStatusSignal.setUpdateFrequencyForAll(1, temp);

        flywheelLeadMotor.optimizeBusUtilization();

        flywheelFollowMotor.setControl(
        new Follower(Constants.CanIDs.SHOOTER_FLYWHEEL_LEAD_CAN_ID, MotorAlignmentValue.Aligned));

        refreshSet =
            new BaseStatusSignal[] {rotorPosition, rotorVelocity, appliedVoltage, current, temp};
  }
    @Override
    public void updateInputs(Inputs inputs) {
        inputs.refreshAll(refreshSet);

        inputs.appliedVolts = appliedVoltage.getValue().in(Units.Volts);
        inputs.currentAmps = current.getValue().in(Units.Amps);
        inputs.tempCelsius = temp.getValue().in(Units.Celsius);
  }
  @Override
  public void setVoltage(double volts) {
    openLoopControl.withOutput(volts);
    flywheelLeadMotor.setControl(openLoopControl);
  }
  @Override
  public void setClosedLoopConstants(
      double kP, double kD, double kG, MotionMagicConfigs mmConfigs) {
    Slot0Configs pidConfig = new Slot0Configs();

    flywheelLeadMotor.getConfigurator().refresh(pidConfig);

    pidConfig.kP = kP;
    pidConfig.kD = kD;
    pidConfig.kG = kG;

    flywheelLeadMotor.getConfigurator().apply(pidConfig);
    flywheelLeadMotor.getConfigurator().apply(mmConfigs);
  }
  @Override
  public boolean setNeutralMode(NeutralModeValue value) {
    var config = new MotorOutputConfigs();

    var status1 = flywheelLeadMotor.getConfigurator().refresh(config);
    var status2 = flywheelFollowMotor.getConfigurator().refresh(config);

    if (status1 != StatusCode.OK || status2 != StatusCode.OK) return false;

    config.NeutralMode = value;

    flywheelLeadMotor.getConfigurator().apply(config);
    flywheelFollowMotor.getConfigurator().apply(config);

    return true;
  }
}