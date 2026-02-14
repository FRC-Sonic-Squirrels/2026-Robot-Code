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
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.MotorConstants.KrakenConstants;

public class ClimberIOReal implements ClimberIO {
  private final StatusSignal<Voltage>         innerClimberAppliedVoltage;
  private final StatusSignal<Angle>           innerClimberPosition;
  private final StatusSignal<Current>         innerClimberCurrent;
  private final StatusSignal<Temperature>     innerClimberTemp;
  private final StatusSignal<AngularVelocity> innerClimberVelocity;
  private final StatusSignal<Voltage>         outerClimberAppliedVoltage;
  private final StatusSignal<Angle>           outerClimberPosition;
  private final StatusSignal<Current>         outerClimberCurrent;
  private final StatusSignal<Temperature>     outerClimberTemp;
  private final StatusSignal<AngularVelocity> outerClimberVelocity;

  private final PositionVoltage innerClimberClosedLoopControl = new PositionVoltage(0.0).withEnableFOC(true);
  private final PositionVoltage outerClimberClosedLoopControl = new PositionVoltage(0.0).withEnableFOC(true);

  private final TalonFX innerClimberMotor = new TalonFX(Constants.CanIDs.CLIMBER_INNER_CAN_ID);
  private final TalonFX outerClimberMotor = new TalonFX(Constants.CanIDs.CLIMBER_OUTER_CAN_ID);

  private final BaseStatusSignal[] climberRefreshSet;

  private Servo climberServo = new Servo(8);

  public ClimberIOReal() {
    // Climber motor config
    TalonFXConfiguration innerClimberConfig = new TalonFXConfiguration();
    TalonFXConfiguration outerClimberConfig = new TalonFXConfiguration();

    innerClimberConfig.CurrentLimits.SupplyCurrentLimit       = ClimberConstants.SUPPLY_CURRENT_LIMIT;
    outerClimberConfig.CurrentLimits.SupplyCurrentLimit       = ClimberConstants.SUPPLY_CURRENT_LIMIT;
    innerClimberConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    outerClimberConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    innerClimberConfig.CurrentLimits.StatorCurrentLimit       = ClimberConstants.STATOR_CURRENT_LIMIT;
    outerClimberConfig.CurrentLimits.StatorCurrentLimit       = ClimberConstants.STATOR_CURRENT_LIMIT;
    innerClimberConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    outerClimberConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    innerClimberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    outerClimberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    innerClimberConfig.MotorOutput.Inverted    = InvertedValue.Clockwise_Positive;
    outerClimberConfig.MotorOutput.Inverted    = InvertedValue.Clockwise_Positive;

    innerClimberConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    outerClimberConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    innerClimberConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    outerClimberConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    innerClimberConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.ClimberConstants.MAX_INNER_CLIMBER_HEIGHT
        * ClimberConstants.INNER_CLIMBER_GEAR_RATIO;
    outerClimberConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.ClimberConstants.MAX_OUTER_CLIMBER_HEIGHT
        * ClimberConstants.OUTER_CLIMBER_GEAR_RATIO;
    innerClimberConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.ClimberConstants.MIN_INNER_CLIMBER_HEIGHT
        * ClimberConstants.INNER_CLIMBER_GEAR_RATIO;
    outerClimberConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.ClimberConstants.MIN_OUTER_CLIMBER_HEIGHT
        * ClimberConstants.OUTER_CLIMBER_GEAR_RATIO;

    innerClimberConfig.Feedback.SensorToMechanismRatio = ClimberConstants.INNER_CLIMBER_GEAR_RATIO;
    outerClimberConfig.Feedback.SensorToMechanismRatio = ClimberConstants.OUTER_CLIMBER_GEAR_RATIO;
    innerClimberConfig.Slot0.GravityType               = GravityTypeValue.Arm_Cosine;
    outerClimberConfig.Slot0.GravityType               = GravityTypeValue.Arm_Cosine;

    innerClimberConfig.Voltage.SupplyVoltageTimeConstant = KrakenConstants.SUPPLY_VOLTAGE_TIME;
    outerClimberConfig.Voltage.SupplyVoltageTimeConstant = KrakenConstants.SUPPLY_VOLTAGE_TIME;

    innerClimberMotor.getConfigurator().apply(innerClimberConfig);
    outerClimberMotor.getConfigurator().apply(outerClimberConfig);
    // Status signals

    innerClimberAppliedVoltage = innerClimberMotor.getMotorVoltage();
    innerClimberPosition       = innerClimberMotor.getPosition();
    innerClimberCurrent        = innerClimberMotor.getStatorCurrent();
    innerClimberTemp           = innerClimberMotor.getDeviceTemp();
    innerClimberVelocity       = innerClimberMotor.getVelocity();

    outerClimberAppliedVoltage = outerClimberMotor.getMotorVoltage();
    outerClimberPosition       = outerClimberMotor.getPosition();
    outerClimberCurrent        = outerClimberMotor.getStatorCurrent();
    outerClimberTemp           = outerClimberMotor.getDeviceTemp();
    outerClimberVelocity       = outerClimberMotor.getVelocity();
    // Update status signals

    BaseStatusSignal.setUpdateFrequencyForAll(100, innerClimberAppliedVoltage, innerClimberPosition,
        innerClimberVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(50, innerClimberCurrent);
    BaseStatusSignal.setUpdateFrequencyForAll(1, innerClimberTemp);

    innerClimberMotor.optimizeBusUtilization();
    outerClimberMotor.optimizeBusUtilization();

    climberRefreshSet = new BaseStatusSignal[] {
        innerClimberAppliedVoltage,
        innerClimberPosition,
        innerClimberCurrent,
        innerClimberTemp,
        innerClimberVelocity,
        outerClimberAppliedVoltage,
        outerClimberPosition,
        outerClimberCurrent,
        outerClimberTemp,
        outerClimberVelocity
    };
  }

  @Override
  public void updateClimberInputs(Inputs inputs) {
    inputs.refreshAll(climberRefreshSet);

    inputs.innerClimberPosition                 = innerClimberPosition.getValue().in(Units.Degrees);
    inputs.innerClimberAppliedVolts             = innerClimberAppliedVoltage.getValue().in(Units.Volts);
    inputs.innerClimberCurrentAmps              = innerClimberCurrent.getValue().in(Units.Amps);
    inputs.innerClimberTempCelsius              = innerClimberTemp.getValue().in(Units.Celsius);
    inputs.innerClimberVelocityDegreesPerSecond = innerClimberVelocity.getValue().in(Units.DegreesPerSecond);
    inputs.outerClimberPosition                 = outerClimberPosition.getValue().in(Units.Degrees);
    inputs.outerClimberAppliedVolts             = outerClimberAppliedVoltage.getValue().in(Units.Volts);
    inputs.outerClimberCurrentAmps              = outerClimberCurrent.getValue().in(Units.Amps);
    inputs.outerClimberTempCelsius              = outerClimberTemp.getValue().in(Units.Celsius);
    inputs.outerClimberVelocityDegreesPerSecond = outerClimberVelocity.getValue().in(Units.DegreesPerSecond);
  }

  // Climber

  @Override
  public void setInnerClimberClosedLoopPosition(double inches) {
    innerClimberClosedLoopControl.withPosition(inches);
    innerClimberMotor.setControl(innerClimberClosedLoopControl);
  }

  @Override
  public void setOuterClimberClosedLoopPosition(double inches) {
    outerClimberClosedLoopControl.withPosition(inches);
    outerClimberMotor.setControl(outerClimberClosedLoopControl);
  }

  @Override
  public void setInnerClimberClosedLoopConstants(double kP, double kD, double kG) {
    var slot0Configs = new Slot0Configs();

    innerClimberMotor.getConfigurator().refresh(slot0Configs);

    slot0Configs.kP = kP;
    slot0Configs.kD = kD;
    slot0Configs.kG = kG;

    innerClimberMotor.getConfigurator().apply(slot0Configs);
  }

  @Override
  public void setOuterClimberClosedLoopConstants(double kP, double kD, double kG) {
    var slot0Configs = new Slot0Configs();

    outerClimberMotor.getConfigurator().refresh(slot0Configs);

    slot0Configs.kP = kP;
    slot0Configs.kD = kD;
    slot0Configs.kG = kG;

    outerClimberMotor.getConfigurator().apply(slot0Configs);
  }

  @Override
  public void resetClimberSensorPosition(Rotation2d angle) {
    innerClimberMotor.setPosition(angle.getRotations());
    outerClimberMotor.setPosition(angle.getRotations());
  }

  @Override
  public boolean setClimberNeutralMode(NeutralModeValue value) {
    var config = new MotorOutputConfigs();

    var status = innerClimberMotor.getConfigurator().refresh(config);

    if (status != StatusCode.OK)
      return false;

    config.NeutralMode = value;

    innerClimberMotor.getConfigurator().apply(config);
    outerClimberMotor.getConfigurator().apply(config);
    return true;
  }

  @Override
  public void setClimberServoAngle(Rotation2d angle) {
    climberServo.setAngle(angle.getDegrees());
  }
}
