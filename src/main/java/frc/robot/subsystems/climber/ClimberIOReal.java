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
import frc.lib.team2930.AutoRegisterLoggers;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.MotorConstants.KrakenConstants;

public class ClimberIOReal implements ClimberIO {

  private final String ROOT_TABLE = "Climber";

  public static class ElevatorLogs extends AutoRegisterLoggers {
    public StatusSignal<Voltage>         appliedVoltage;
    public StatusSignal<Current>         currentAmps;
    public StatusSignal<Temperature>     tempCelsius;
    public StatusSignal<AngularVelocity> velocityDegreesPerSecond;
    public StatusSignal<Angle>           targetHeightInches;
  }

  public static class Logs extends AutoRegisterLoggers {
    public ElevatorLogs innerClimber;
    public ElevatorLogs outerClimber;

    public LoggerEntry.Decimal servoTargetAngle;

  }

  public final Logs logs = LoggerGroup.root.newAutoLogger(Logs.class, ROOT_TABLE);

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
    logs.innerClimber.appliedVoltage           = innerClimberMotor.getMotorVoltage();
    logs.innerClimber.targetHeightInches       = innerClimberMotor.getPosition();
    logs.innerClimber.currentAmps              = innerClimberMotor.getStatorCurrent();
    logs.innerClimber.tempCelsius              = innerClimberMotor.getDeviceTemp();
    logs.innerClimber.velocityDegreesPerSecond = innerClimberMotor.getVelocity();
    logs.outerClimber.appliedVoltage           = outerClimberMotor.getMotorVoltage();
    logs.outerClimber.targetHeightInches       = outerClimberMotor.getPosition();
    logs.outerClimber.currentAmps              = outerClimberMotor.getStatorCurrent();
    logs.outerClimber.tempCelsius              = outerClimberMotor.getDeviceTemp();
    logs.outerClimber.velocityDegreesPerSecond = outerClimberMotor.getVelocity();
    // Update status signals

    BaseStatusSignal.setUpdateFrequencyForAll(100, logs.innerClimber.appliedVoltage,
        logs.innerClimber.targetHeightInches, logs.innerClimber.velocityDegreesPerSecond);
    BaseStatusSignal.setUpdateFrequencyForAll(50, logs.innerClimber.currentAmps);
    BaseStatusSignal.setUpdateFrequencyForAll(1, logs.innerClimber.tempCelsius);

    BaseStatusSignal.setUpdateFrequencyForAll(100, logs.outerClimber.appliedVoltage,
        logs.outerClimber.targetHeightInches, logs.outerClimber.velocityDegreesPerSecond);
    BaseStatusSignal.setUpdateFrequencyForAll(50, logs.outerClimber.currentAmps);
    BaseStatusSignal.setUpdateFrequencyForAll(1, logs.outerClimber.tempCelsius);

    innerClimberMotor.optimizeBusUtilization();
    outerClimberMotor.optimizeBusUtilization();

    climberRefreshSet = new BaseStatusSignal[] {
        logs.innerClimber.appliedVoltage, logs.innerClimber.targetHeightInches,
        logs.innerClimber.currentAmps, logs.innerClimber.tempCelsius, logs.innerClimber.velocityDegreesPerSecond,
        logs.outerClimber.appliedVoltage, logs.outerClimber.targetHeightInches, logs.outerClimber.currentAmps,
        logs.outerClimber.tempCelsius, logs.outerClimber.velocityDegreesPerSecond
    };
  }

  @Override
  public void updateClimberInputs(Inputs inputs) {
    inputs.refreshAll(climberRefreshSet);

    inputs.innerClimberPosition                 = logs.innerClimber.targetHeightInches.getValue().in(Units.Degrees);
    inputs.innerClimberAppliedVolts             = logs.innerClimber.appliedVoltage.getValue().in(Units.Volts);
    inputs.innerClimberCurrentAmps              = logs.innerClimber.currentAmps.getValue().in(Units.Amps);
    inputs.innerClimberTempCelsius              = logs.innerClimber.tempCelsius.getValue().in(Units.Celsius);
    inputs.innerClimberVelocityDegreesPerSecond = logs.innerClimber.velocityDegreesPerSecond.getValue()
        .in(Units.DegreesPerSecond);
    inputs.outerClimberPosition                 = logs.outerClimber.targetHeightInches.getValue().in(Units.Degrees);
    inputs.outerClimberAppliedVolts             = logs.outerClimber.appliedVoltage.getValue().in(Units.Volts);
    inputs.outerClimberCurrentAmps              = logs.outerClimber.currentAmps.getValue().in(Units.Amps);
    inputs.outerClimberTempCelsius              = logs.outerClimber.tempCelsius.getValue().in(Units.Celsius);
    inputs.outerClimberVelocityDegreesPerSecond = logs.outerClimber.velocityDegreesPerSecond.getValue()
        .in(Units.DegreesPerSecond);
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
