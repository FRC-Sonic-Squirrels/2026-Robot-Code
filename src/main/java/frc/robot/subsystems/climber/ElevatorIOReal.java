package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.MotorConstants.KrakenConstants;

public class ElevatorIOReal implements ElevatorIO {
  private final PositionVoltage closedLoopControl = new PositionVoltage(0.0).withEnableFOC(true);

  public StatusSignal<Voltage>         appliedVoltage;
  public StatusSignal<Current>         currentAmps;
  public StatusSignal<Temperature>     tempCelsius;
  public StatusSignal<AngularVelocity> velocityDegreesPerSecond;
  public StatusSignal<Angle>           targetHeightInches;

  private final BaseStatusSignal[] refreshSet;

  private final TalonFX motor;

  public ElevatorIOReal(ElevatorIOConfig config) {
    this.motor = new TalonFX(config.motorCANID());

    // Elevator motor config
    TalonFXConfiguration configFx = new TalonFXConfiguration();

    configFx.CurrentLimits.SupplyCurrentLimit              = ClimberConstants.SUPPLY_CURRENT_LIMIT;
    configFx.CurrentLimits.SupplyCurrentLimitEnable        = true;
    configFx.CurrentLimits.StatorCurrentLimit              = ClimberConstants.STATOR_CURRENT_LIMIT;
    configFx.CurrentLimits.StatorCurrentLimitEnable        = true;
    configFx.MotorOutput.NeutralMode                       = NeutralModeValue.Brake;
    configFx.MotorOutput.Inverted                          = InvertedValue.Clockwise_Positive;
    configFx.SoftwareLimitSwitch.ForwardSoftLimitEnable    = true;
    configFx.SoftwareLimitSwitch.ReverseSoftLimitEnable    = true;
    configFx.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.ClimberConstants.MAX_INNER_CLIMBER_HEIGHT
        * ClimberConstants.INNER_CLIMBER_GEAR_RATIO;
    configFx.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.ClimberConstants.MIN_INNER_CLIMBER_HEIGHT
        * ClimberConstants.INNER_CLIMBER_GEAR_RATIO;

    configFx.Feedback.SensorToMechanismRatio   = ClimberConstants.INNER_CLIMBER_GEAR_RATIO;
    configFx.Slot0.GravityType                 = GravityTypeValue.Arm_Cosine;
    configFx.Voltage.SupplyVoltageTimeConstant = KrakenConstants.SUPPLY_VOLTAGE_TIME;

    motor.getConfigurator().apply(configFx);
    // Status signals
    appliedVoltage           = motor.getMotorVoltage();
    targetHeightInches       = motor.getPosition();
    currentAmps              = motor.getStatorCurrent();
    tempCelsius              = motor.getDeviceTemp();
    velocityDegreesPerSecond = motor.getVelocity();
    // Update status signals

    BaseStatusSignal.setUpdateFrequencyForAll(50, appliedVoltage, targetHeightInches, velocityDegreesPerSecond);
    BaseStatusSignal.setUpdateFrequencyForAll(50, currentAmps);
    BaseStatusSignal.setUpdateFrequencyForAll(1, tempCelsius);

    motor.optimizeBusUtilization();

    refreshSet = new BaseStatusSignal[] {
        appliedVoltage,
        targetHeightInches,
        currentAmps,
        tempCelsius,
        velocityDegreesPerSecond
    };
  }

  @Override
  public void updateElevatorInputs(Inputs inputs) {
    inputs.refreshAll(refreshSet);

    inputs.position                 = targetHeightInches.getValue().in(Units.Degrees);
    inputs.appliedVolts             = appliedVoltage.getValue().in(Units.Volts);
    inputs.currentAmps              = currentAmps.getValue().in(Units.Amps);
    inputs.tempCelsius              = tempCelsius.getValue().in(Units.Celsius);
    inputs.velocityDegreesPerSecond = velocityDegreesPerSecond.getValue().in(Units.DegreesPerSecond);
  }

  // Climber

  @Override
  public void setElevatorClosedLoopPosition(double inches) {
    closedLoopControl.withPosition(inches);
    motor.setControl(closedLoopControl);
  }

  @Override
  public void setElevatorClosedLoopConstants(double kP, double kD, double kG) {
    var slot0Configs = new Slot0Configs();

    motor.getConfigurator().refresh(slot0Configs);

    slot0Configs.kP = kP;
    slot0Configs.kD = kD;
    slot0Configs.kG = kG;

    motor.getConfigurator().apply(slot0Configs);
  }

  @Override
  public void resetElevatorSensorPosition(Rotation2d angle) {
    motor.setPosition(angle.getRotations());
  }

  @Override
  public boolean setElevatorNeutralMode(NeutralModeValue value) {
    var config = new MotorOutputConfigs();

    var status = motor.getConfigurator().refresh(config);

    if (status != StatusCode.OK)
      return false;

    config.NeutralMode = value;

    motor.getConfigurator().apply(config);
    return true;
  }
}
