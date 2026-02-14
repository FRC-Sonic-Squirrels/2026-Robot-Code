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
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.MotorConstants.KrakenConstants;

public class ElevatorIOReal implements ElevatorIO {

  private final String ROOT_TABLE = "Elevator";

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

  public static class Logs extends AutoRegisterLoggers {
    public ElevatorLogs elevatorLogs;

    public LoggerEntry.Decimal servoTargetAngle;

  }

  public final Logs logs = LoggerGroup.root.newAutoLogger(Logs.class, ROOT_TABLE);

  private final PositionVoltage closedLoopControl = new PositionVoltage(0.0).withEnableFOC(true);

  private final BaseStatusSignal[] climberRefreshSet;

  private TalonFX elevatorMotor;

  public ElevatorIOReal(TalonFX elevatorMotor) {
    this.elevatorMotor = elevatorMotor;
    // Elevator motor config
    TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();

    elevatorConfig.CurrentLimits.SupplyCurrentLimit              = ClimberConstants.SUPPLY_CURRENT_LIMIT;
    elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable        = true;
    elevatorConfig.CurrentLimits.StatorCurrentLimit              = ClimberConstants.STATOR_CURRENT_LIMIT;
    elevatorConfig.CurrentLimits.StatorCurrentLimitEnable        = true;
    elevatorConfig.MotorOutput.NeutralMode                       = NeutralModeValue.Brake;
    elevatorConfig.MotorOutput.Inverted                          = InvertedValue.Clockwise_Positive;
    elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable    = true;
    elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable    = true;
    elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.ClimberConstants.MAX_INNER_CLIMBER_HEIGHT
        * ClimberConstants.INNER_CLIMBER_GEAR_RATIO;
    elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.ClimberConstants.MIN_INNER_CLIMBER_HEIGHT
        * ClimberConstants.INNER_CLIMBER_GEAR_RATIO;

    elevatorConfig.Feedback.SensorToMechanismRatio = ClimberConstants.INNER_CLIMBER_GEAR_RATIO;
    elevatorConfig.Slot0.GravityType               = GravityTypeValue.Arm_Cosine;

    elevatorConfig.Voltage.SupplyVoltageTimeConstant = KrakenConstants.SUPPLY_VOLTAGE_TIME;

    elevatorMotor.getConfigurator().apply(elevatorConfig);
    // Status signals
    logs.elevatorLogs.appliedVoltage           = elevatorMotor.getMotorVoltage();
    logs.elevatorLogs.targetHeightInches       = elevatorMotor.getPosition();
    logs.elevatorLogs.currentAmps              = elevatorMotor.getStatorCurrent();
    logs.elevatorLogs.tempCelsius              = elevatorMotor.getDeviceTemp();
    logs.elevatorLogs.velocityDegreesPerSecond = elevatorMotor.getVelocity();
    // Update status signals

    BaseStatusSignal.setUpdateFrequencyForAll(100, logs.elevatorLogs.appliedVoltage,
        logs.elevatorLogs.targetHeightInches, logs.elevatorLogs.velocityDegreesPerSecond);
    BaseStatusSignal.setUpdateFrequencyForAll(50, logs.elevatorLogs.currentAmps);
    BaseStatusSignal.setUpdateFrequencyForAll(1, logs.elevatorLogs.tempCelsius);

    elevatorMotor.optimizeBusUtilization();

    climberRefreshSet = new BaseStatusSignal[] { logs.elevatorLogs.appliedVoltage, logs.elevatorLogs.targetHeightInches,
        logs.elevatorLogs.currentAmps, logs.elevatorLogs.tempCelsius, logs.elevatorLogs.velocityDegreesPerSecond };
  }

  @Override
  public void updateElevatorInputs(Inputs inputs) {
    inputs.refreshAll(climberRefreshSet);

    inputs.position                 = logs.elevatorLogs.targetHeightInches.getValue().in(Units.Degrees);
    inputs.appliedVolts             = logs.elevatorLogs.appliedVoltage.getValue().in(Units.Volts);
    inputs.currentAmps              = logs.elevatorLogs.currentAmps.getValue().in(Units.Amps);
    inputs.tempCelsius              = logs.elevatorLogs.tempCelsius.getValue().in(Units.Celsius);
    inputs.velocityDegreesPerSecond = logs.elevatorLogs.velocityDegreesPerSecond.getValue().in(Units.DegreesPerSecond);
  }

  // Climber

  @Override
  public void setElevatorClosedLoopPosition(double inches) {
    closedLoopControl.withPosition(inches);
    elevatorMotor.setControl(closedLoopControl);
  }

  @Override
  public void setElevatorClosedLoopConstants(double kP, double kD, double kG) {
    var slot0Configs = new Slot0Configs();

    elevatorMotor.getConfigurator().refresh(slot0Configs);

    slot0Configs.kP = kP;
    slot0Configs.kD = kD;
    slot0Configs.kG = kG;

    elevatorMotor.getConfigurator().apply(slot0Configs);
  }

  @Override
  public void resetElevatorSensorPosition(Rotation2d angle) {
    elevatorMotor.setPosition(angle.getRotations());
  }

  @Override
  public boolean setElevatorNeutralMode(NeutralModeValue value) {
    var config = new MotorOutputConfigs();

    var status = elevatorMotor.getConfigurator().refresh(config);

    if (status != StatusCode.OK)
      return false;

    config.NeutralMode = value;

    elevatorMotor.getConfigurator().apply(config);
    return true;
  }
}
