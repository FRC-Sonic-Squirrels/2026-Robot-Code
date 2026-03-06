package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.DIOPorts;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOReal implements ShooterIO {
  private final TalonFX flywheelLead;
  private final TalonFX flywheelFollow;
  private final TalonFX hood;
  private final TalonFX kicker;

  private final DigitalInput kickerBeamBreak = new DigitalInput(DIOPorts.SHOOTER_BEAM_BREAKER_DOI_ID);

  private final StatusSignal<Angle>           hoodPosition;
  private final StatusSignal<AngularVelocity> hoodVelocity;
  private final StatusSignal<Voltage>         hoodVoltage;
  private final StatusSignal<Current>         hoodCurrentAmps;

  private final StatusSignal<AngularVelocity> flywheelLeadVelocity;
  private final StatusSignal<Voltage>         flywheelLeadVoltage;
  private final StatusSignal<Current>         flywheelLeadCurrentAmps;
  private final StatusSignal<AngularVelocity> flywheelFollowVelocity;
  private final StatusSignal<Voltage>         flywheelFollowerVoltage;
  private final StatusSignal<Current>         flywheelFollowerCurrentAmps;

  private final StatusSignal<Voltage>         kickerAppliedVolts;
  private final StatusSignal<Current>         kickerCurrentAmps;
  private final StatusSignal<AngularVelocity> kickerVelocity;

  private final StatusSignal<Temperature> flywheelLeadTempCelsius;
  private final StatusSignal<Temperature> flywheelFollowTempCelsius;
  private final StatusSignal<Temperature> hoodTempCelsius;
  private final StatusSignal<Temperature> kickerTempCelsius;

  private final BaseStatusSignal[] refreshSet;

  private final MotionMagicVoltage         hoodClosedLoopControl = new MotionMagicVoltage(0).withEnableFOC(true);
  private final MotionMagicVelocityVoltage flywheelClosedLoop    = new MotionMagicVelocityVoltage(0.0)
      .withEnableFOC(true);

  private final MotionMagicVelocityVoltage kickerClosedLoop = new MotionMagicVelocityVoltage(0).withEnableFOC(true);
  private double                           kickerTargetRPM;

  public ShooterIOReal(TalonFX flywheelLead, TalonFX flywheelFollow, TalonFX hood, TalonFX kicker) {
    this.flywheelLead   = flywheelLead;
    this.flywheelFollow = flywheelFollow;
    this.hood           = hood;
    this.kicker         = kicker;

    // --- flywheel config ---
    TalonFXConfiguration flywheelLeadConfig = new TalonFXConfiguration();
    flywheelLeadConfig.CurrentLimits.SupplyCurrentLimit       = ShooterConstants.Flywheel.SUPPLY_CURRENT_LIMIT;
    flywheelLeadConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    flywheelLeadConfig.CurrentLimits.StatorCurrentLimit       = ShooterConstants.Flywheel.STATOR_CURRENT_LIMIT;
    flywheelLeadConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    flywheelLeadConfig.MotorOutput.NeutralMode         = NeutralModeValue.Coast;
    flywheelLeadConfig.MotorOutput.Inverted            = InvertedValue.CounterClockwise_Positive;
    flywheelLeadConfig.Feedback.SensorToMechanismRatio = ShooterConstants.Flywheel.GEARING;

    flywheelLeadConfig.Voltage.PeakReverseVoltage        = 0.0;
    flywheelLeadConfig.Voltage.SupplyVoltageTimeConstant = 0.02;

    TalonFXConfiguration flywheelFollowerConfig = new TalonFXConfiguration();
    flywheelFollowerConfig.CurrentLimits.SupplyCurrentLimit       = ShooterConstants.Flywheel.SUPPLY_CURRENT_LIMIT;
    flywheelFollowerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    flywheelFollowerConfig.CurrentLimits.StatorCurrentLimit       = ShooterConstants.Flywheel.STATOR_CURRENT_LIMIT;
    flywheelFollowerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    flywheelFollowerConfig.MotorOutput.NeutralMode         = NeutralModeValue.Coast;
    flywheelFollowerConfig.MotorOutput.Inverted            = InvertedValue.Clockwise_Positive;
    flywheelFollowerConfig.Feedback.SensorToMechanismRatio = ShooterConstants.Flywheel.GEARING;

    flywheelFollowerConfig.Voltage.PeakReverseVoltage        = 0.0;
    flywheelFollowerConfig.Voltage.SupplyVoltageTimeConstant = 0.02;

    flywheelLead.getConfigurator().apply(flywheelLeadConfig);
    flywheelFollow.getConfigurator().apply(flywheelFollowerConfig);

    // --- hood config ---
    TalonFXConfiguration hoodConfig = new TalonFXConfiguration();

    hoodConfig.Feedback.SensorToMechanismRatio = Constants.ShooterConstants.Hood.GEARING;
    hoodConfig.MotorOutput.NeutralMode         = NeutralModeValue.Brake;

    hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.ShooterConstants.Hood.MAX_ANGLE_RAD
        .getRotations();
    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.ShooterConstants.Hood.MIN_ANGLE_RAD
        .getRotations();

    hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    hoodConfig.CurrentLimits.SupplyCurrentLimit       = ShooterConstants.Hood.SUPPLY_CURRENT_LIMIT;
    hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    hoodConfig.CurrentLimits.StatorCurrentLimit       = ShooterConstants.Hood.STATOR_CURRENT_LIMIT;
    hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    hood.getConfigurator().apply(hoodConfig);

    // -- kicker config --
    TalonFXConfiguration kickerConfig = new TalonFXConfiguration();

    kickerConfig.CurrentLimits.SupplyCurrentLimit       = ShooterConstants.Kicker.SUPPLY_CURRENT_LIMIT;
    kickerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    kickerConfig.CurrentLimits.StatorCurrentLimit       = ShooterConstants.Kicker.STATOR_CURRENT_LIMIT;
    kickerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    kickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    kickerConfig.Feedback.SensorToMechanismRatio = ShooterConstants.Kicker.GEARING;

    kicker.getConfigurator().apply(kickerConfig);

    hoodPosition    = hood.getPosition();
    hoodVelocity    = hood.getVelocity();
    hoodVoltage     = hood.getMotorVoltage();
    hoodCurrentAmps = hood.getStatorCurrent();

    flywheelLeadVelocity   = flywheelLead.getVelocity();
    flywheelFollowVelocity = flywheelFollow.getVelocity();

    flywheelLeadVoltage         = flywheelLead.getMotorVoltage();
    flywheelFollowerVoltage     = flywheelFollow.getMotorVoltage();
    flywheelLeadCurrentAmps     = flywheelLead.getStatorCurrent();
    flywheelFollowerCurrentAmps = flywheelFollow.getStatorCurrent();

    kickerAppliedVolts = kicker.getMotorVoltage();
    kickerCurrentAmps  = kicker.getStatorCurrent();
    kickerVelocity     = kicker.getVelocity();

    flywheelLeadTempCelsius   = flywheelLead.getDeviceTemp();
    flywheelFollowTempCelsius = flywheelLead.getDeviceTemp();
    hoodTempCelsius           = hood.getDeviceTemp();
    kickerTempCelsius         = kicker.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(50, hoodPosition, flywheelLeadVelocity, flywheelFollowVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(50, hoodVelocity, hoodVoltage, hoodCurrentAmps, kickerAppliedVolts,
        kickerCurrentAmps, kickerVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(10, flywheelLeadVoltage, flywheelLeadCurrentAmps, flywheelFollowerVoltage,
        flywheelFollowerCurrentAmps);
    BaseStatusSignal.setUpdateFrequencyForAll(1, flywheelLeadTempCelsius, flywheelFollowTempCelsius, kickerTempCelsius,
        hoodTempCelsius);

    flywheelLead.optimizeBusUtilization();
    flywheelFollow.optimizeBusUtilization();
    hood.optimizeBusUtilization();
    kicker.optimizeBusUtilization();

    refreshSet = new BaseStatusSignal[] {
        hoodPosition,
        hoodVelocity,
        hoodVoltage,
        hoodCurrentAmps,
        // --
        flywheelLeadVelocity,
        flywheelLeadVoltage,
        flywheelLeadCurrentAmps,
        flywheelFollowVelocity,
        flywheelFollowerVoltage,
        flywheelFollowerCurrentAmps,
        // --
        kickerAppliedVolts,
        kickerCurrentAmps,
        kickerVelocity,
        // --
        flywheelLeadTempCelsius,
        flywheelFollowTempCelsius,
        hoodTempCelsius,
        kickerTempCelsius,
    };
  }

  @Override
  public void updateInputs(Inputs inputs) {
    inputs.refreshAll(refreshSet);

    inputs.hoodPosition = Rotation2d.fromRotations(hoodPosition.getValueAsDouble());
    // rotations to rads = mult by 2pi. 1 full rot = 2pi rads
    inputs.hoodVelocityRadsPerSec = hoodVelocity.getValueAsDouble() * (2 * Math.PI);
    inputs.hoodAppliedVolts       = hoodVoltage.getValue().in(Units.Volts);
    inputs.hoodCurrentAmps        = hoodCurrentAmps.getValue().in(Units.Amps);

    inputs.flywheelRPM[0] = flywheelLeadVelocity.getValue().in(Units.RPM);
    inputs.flywheelRPM[1] = flywheelFollowVelocity.getValue().in(Units.RPM);

    // rps to rpm = mult by 60
    inputs.flywheelAppliedVolts[0] = flywheelLeadVoltage.getValue().in(Units.Volts);
    inputs.flywheelAppliedVolts[1] = flywheelFollowerVoltage.getValue().in(Units.Volts);

    inputs.flywheelCurrentAmps[0] = flywheelLeadCurrentAmps.getValue().in(Units.Amps);
    inputs.flywheelCurrentAmps[1] = flywheelFollowerCurrentAmps.getValue().in(Units.Amps);

    inputs.kickerAppliedVolts = kickerAppliedVolts.getValue().in(Units.Volts);
    inputs.kickerCurrentAmps  = kickerCurrentAmps.getValue().in(Units.Amps);
    inputs.kickerVelocityRPM  = kickerVelocity.getValue().in(Units.RPM);
    inputs.kickerTargetRPM    = kickerTargetRPM;

    inputs.tempsCelcius[0] = flywheelLeadTempCelsius.getValue().in(Units.Celsius);
    inputs.tempsCelcius[1] = flywheelFollowTempCelsius.getValue().in(Units.Celsius);
    inputs.tempsCelcius[2] = hoodTempCelsius.getValue().in(Units.Celsius);
    inputs.tempsCelcius[3] = kickerTempCelsius.getValue().in(Units.Celsius);
  }

  // HOOD

  @Override
  public void setHoodPosition(Rotation2d rot) {
    hoodClosedLoopControl.withPosition(rot.getRotations());
    hood.setControl(hoodClosedLoopControl);
  }

  // Flywheel

  @Override
  public void setFlywheelRPM(double revPerMin) {
    flywheelLead.setControl(flywheelClosedLoop.withVelocity(revPerMin / 60));
  }

  @Override
  public void setHoodClosedLoopConstants(double kP, double kD, double kG, double maxProfiledVelocity,
      double maxProfiledAcceleration) {
    Slot0Configs       pidConfig = new Slot0Configs();
    MotionMagicConfigs mmConfig  = new MotionMagicConfigs();

    var configurator = hood.getConfigurator();
    configurator.refresh(pidConfig);
    configurator.refresh(mmConfig);

    pidConfig.kP = kP;
    pidConfig.kD = kD;
    pidConfig.kG = kG;

    mmConfig.MotionMagicCruiseVelocity = maxProfiledVelocity;
    mmConfig.MotionMagicAcceleration   = maxProfiledAcceleration;

    configurator.apply(pidConfig);
    configurator.apply(mmConfig);
  }

  @Override
  public void setFlywheelClosedLoopConstants(double kP, double kV, double kS, double maxProfiledAcceleration) {
    Slot0Configs       pidConfig = new Slot0Configs();
    MotionMagicConfigs mmConfig  = new MotionMagicConfigs();

    var leadConfigurator   = flywheelLead.getConfigurator();
    var followConfigurator = flywheelFollow.getConfigurator();

    leadConfigurator.refresh(pidConfig);
    leadConfigurator.refresh(mmConfig);

    followConfigurator.refresh(pidConfig);
    followConfigurator.refresh(mmConfig);

    pidConfig.kP = kP;
    pidConfig.kV = kV;
    pidConfig.kS = kS;

    mmConfig.MotionMagicAcceleration = maxProfiledAcceleration;

    leadConfigurator.apply(pidConfig);
    leadConfigurator.apply(mmConfig);

    followConfigurator.apply(pidConfig);
    followConfigurator.apply(mmConfig);
  }

  @Override
  public boolean setNeutralMode(NeutralModeValue value) {
    var config = new MotorOutputConfigs();

    var status = hood.getConfigurator().refresh(config);

    if (status != StatusCode.OK) {
      return false;
    }

    config.NeutralMode = value;

    hood.getConfigurator().apply(config);
    return true;
  }

  @Override
  public void setKickerClosedLoopConstants(double kP, double kV, double kS, double maxProfiledAcceleration) {
    Slot0Configs       pidConfig = new Slot0Configs();
    MotionMagicConfigs mmConfig  = new MotionMagicConfigs();

    var config = kicker.getConfigurator();

    config.refresh(pidConfig);
    config.refresh(mmConfig);

    pidConfig.kP = kP;
    pidConfig.kV = kV;
    pidConfig.kS = kS;

    mmConfig.MotionMagicAcceleration = maxProfiledAcceleration;

    config.apply(pidConfig);
    config.apply(mmConfig);
  }

  @Override
  public void setKickerVelocity(double revPerMin) {
    kickerTargetRPM = revPerMin;
    kicker.setControl(kickerClosedLoop.withVelocity(revPerMin / 60.0));
  }
}
