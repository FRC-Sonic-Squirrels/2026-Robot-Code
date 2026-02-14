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
import frc.lib.team2930.AutoRegisterLoggers;
import frc.lib.team2930.LoggerGroup;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.MotorConstants.KrakenConstants;

public class ElevatorIOReal implements ElevatorIO {

  public static class ElevatorLogs extends AutoRegisterLoggers {
    public StatusSignal<Voltage>         appliedVoltage;
    public StatusSignal<Current>         currentAmps;
    public StatusSignal<Temperature>     tempCelsius;
    public StatusSignal<AngularVelocity> velocityDegreesPerSecond;
    /**
     * @deprecated This field actually stores an angular position in degrees, not a
     *             height in inches. Use {@link #angleDegrees} instead.
     */
    @Deprecated
    public StatusSignal<Angle>           targetHeightInches;
    // Correctly named alias for the climber's angular position in degrees.
    public StatusSignal<Angle> angleDegrees;
  }

  public final ElevatorLogs logs;

  private final PositionVoltage closedLoopControl = new PositionVoltage(0.0).withEnableFOC(true);

  private final BaseStatusSignal[] refreshSet;

  private TalonFX motor;

  public ElevatorIOReal(String name, TalonFX motor) {
    this.motor = motor;
    this.logs  = LoggerGroup.root.newAutoLogger(ElevatorLogs.class, name);
    // Elevator motor config
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit              = ClimberConstants.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable        = true;
    config.CurrentLimits.StatorCurrentLimit              = ClimberConstants.STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable        = true;
    config.MotorOutput.NeutralMode                       = NeutralModeValue.Brake;
    config.MotorOutput.Inverted                          = InvertedValue.Clockwise_Positive;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable    = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable    = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.ClimberConstants.MAX_INNER_CLIMBER_HEIGHT
        * ClimberConstants.INNER_CLIMBER_GEAR_RATIO;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.ClimberConstants.MIN_INNER_CLIMBER_HEIGHT
        * ClimberConstants.INNER_CLIMBER_GEAR_RATIO;

    config.Feedback.SensorToMechanismRatio   = ClimberConstants.INNER_CLIMBER_GEAR_RATIO;
    config.Slot0.GravityType                 = GravityTypeValue.Arm_Cosine;
    config.Voltage.SupplyVoltageTimeConstant = KrakenConstants.SUPPLY_VOLTAGE_TIME;

    motor.getConfigurator().apply(config);
    // Status signals
    logs.appliedVoltage           = motor.getMotorVoltage();
    logs.targetHeightInches       = motor.getPosition();
    logs.currentAmps              = motor.getStatorCurrent();
    logs.tempCelsius              = motor.getDeviceTemp();
    logs.velocityDegreesPerSecond = motor.getVelocity();
    // Update status signals

    BaseStatusSignal.setUpdateFrequencyForAll(100, logs.appliedVoltage, logs.targetHeightInches,
        logs.velocityDegreesPerSecond);
    BaseStatusSignal.setUpdateFrequencyForAll(50, logs.currentAmps);
    BaseStatusSignal.setUpdateFrequencyForAll(1, logs.tempCelsius);

    motor.optimizeBusUtilization();

    refreshSet = new BaseStatusSignal[] { logs.appliedVoltage, logs.targetHeightInches, logs.currentAmps,
        logs.tempCelsius, logs.velocityDegreesPerSecond };
  }

  @Override
  public void updateElevatorInputs(Inputs inputs) {
    inputs.refreshAll(refreshSet);

    inputs.position                 = logs.targetHeightInches.getValue().in(Units.Degrees);
    inputs.appliedVolts             = logs.appliedVoltage.getValue().in(Units.Volts);
    inputs.currentAmps              = logs.currentAmps.getValue().in(Units.Amps);
    inputs.tempCelsius              = logs.tempCelsius.getValue().in(Units.Celsius);
    inputs.velocityDegreesPerSecond = logs.velocityDegreesPerSecond.getValue().in(Units.DegreesPerSecond);
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
