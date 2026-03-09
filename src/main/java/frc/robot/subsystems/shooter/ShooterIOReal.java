package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.DIOPorts;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.TalonFXWrapper;
import frc.robot.configs.RobotConfig2026;

import java.util.function.Consumer;

public class ShooterIOReal implements ShooterIO {
  private final TalonFXWrapper flywheelLead;
  private final TalonFXWrapper flywheelFollow;
  private final TalonFXWrapper hood;
  private final TalonFXWrapper kicker;

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

  public ShooterIOReal(TalonFXWrapper flywheelLead, TalonFXWrapper flywheelFollow, TalonFXWrapper hood,
      TalonFXWrapper kicker) {
    this.flywheelLead   = flywheelLead;
    this.flywheelFollow = flywheelFollow;
    this.hood           = hood;
    this.kicker         = kicker;

    // --- flywheel config ---
    flywheelLead.talonFXConfiguration.change((flywheelLeadConfig) -> {
      flywheelLeadConfig.CurrentLimits.SupplyCurrentLimit       = ShooterConstants.Flywheel.SUPPLY_CURRENT_LIMIT;
      flywheelLeadConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
      flywheelLeadConfig.CurrentLimits.StatorCurrentLimit       = ShooterConstants.Flywheel.STATOR_CURRENT_LIMIT;
      flywheelLeadConfig.CurrentLimits.StatorCurrentLimitEnable = true;

      flywheelLeadConfig.MotorOutput.NeutralMode         = NeutralModeValue.Coast;
      flywheelLeadConfig.MotorOutput.Inverted            = InvertedValue.CounterClockwise_Positive;
      flywheelLeadConfig.Feedback.SensorToMechanismRatio = ShooterConstants.Flywheel.GEARING;

      flywheelLeadConfig.Voltage.PeakReverseVoltage        = 0.0;
      flywheelLeadConfig.Voltage.SupplyVoltageTimeConstant = 0.02;
    });

    flywheelFollow.talonFXConfiguration.change((flywheelFollowerConfig) -> {
      flywheelFollowerConfig.CurrentLimits.SupplyCurrentLimit       = ShooterConstants.Flywheel.SUPPLY_CURRENT_LIMIT;
      flywheelFollowerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
      flywheelFollowerConfig.CurrentLimits.StatorCurrentLimit       = ShooterConstants.Flywheel.STATOR_CURRENT_LIMIT;
      flywheelFollowerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

      flywheelFollowerConfig.MotorOutput.NeutralMode         = NeutralModeValue.Coast;
      flywheelFollowerConfig.MotorOutput.Inverted            = InvertedValue.Clockwise_Positive;
      flywheelFollowerConfig.Feedback.SensorToMechanismRatio = ShooterConstants.Flywheel.GEARING;

      flywheelFollowerConfig.Voltage.PeakReverseVoltage        = 0.0;
      flywheelFollowerConfig.Voltage.SupplyVoltageTimeConstant = 0.02;
    });

    // --- hood config ---
    hood.talonFXConfiguration.change((hoodConfig) -> {
      hoodConfig.Feedback.SensorToMechanismRatio = Constants.ShooterConstants.Hood.GEARING;
      hoodConfig.MotorOutput.NeutralMode         = NeutralModeValue.Brake;
      hoodConfig.MotorOutput.Inverted            = InvertedValue.Clockwise_Positive;

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
    });

    hood.setNeutralMode(NeutralModeValue.Coast);

    // -- kicker config --
    kicker.talonFXConfiguration.change((kickerConfig) -> {
      kickerConfig.CurrentLimits.SupplyCurrentLimit       = ShooterConstants.Kicker.SUPPLY_CURRENT_LIMIT;
      kickerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
      kickerConfig.CurrentLimits.StatorCurrentLimit       = ShooterConstants.Kicker.STATOR_CURRENT_LIMIT;
      kickerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

      kickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

      kickerConfig.Feedback.SensorToMechanismRatio = ShooterConstants.Kicker.GEARING;
    });

    var flywheelLeadIO   = flywheelLead.io;
    var flywheelFollowIO = flywheelFollow.io;
    var hoodIO           = hood.io;
    var kickerIO         = kicker.io;

    hoodPosition    = hoodIO.getPosition();
    hoodVelocity    = hoodIO.getVelocity();
    hoodVoltage     = hoodIO.getMotorVoltage();
    hoodCurrentAmps = hoodIO.getStatorCurrent();

    flywheelLeadVelocity   = flywheelLeadIO.getVelocity();
    flywheelFollowVelocity = flywheelFollowIO.getVelocity();

    flywheelLeadVoltage         = flywheelLeadIO.getMotorVoltage();
    flywheelFollowerVoltage     = flywheelFollowIO.getMotorVoltage();
    flywheelLeadCurrentAmps     = flywheelLeadIO.getStatorCurrent();
    flywheelFollowerCurrentAmps = flywheelFollowIO.getStatorCurrent();

    kickerAppliedVolts = kickerIO.getMotorVoltage();
    kickerCurrentAmps  = kickerIO.getStatorCurrent();
    kickerVelocity     = kickerIO.getVelocity();

    flywheelLeadTempCelsius   = flywheelLeadIO.getDeviceTemp();
    flywheelFollowTempCelsius = flywheelLeadIO.getDeviceTemp();
    hoodTempCelsius           = hoodIO.getDeviceTemp();
    kickerTempCelsius         = kickerIO.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(50, hoodPosition, flywheelLeadVelocity, flywheelFollowVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(50, hoodVelocity, hoodVoltage, hoodCurrentAmps, kickerAppliedVolts,
        kickerCurrentAmps, kickerVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(10, flywheelLeadVoltage, flywheelLeadCurrentAmps, flywheelFollowerVoltage,
        flywheelFollowerCurrentAmps);
    BaseStatusSignal.setUpdateFrequencyForAll(1, flywheelLeadTempCelsius, flywheelFollowTempCelsius, kickerTempCelsius,
        hoodTempCelsius);

    flywheelLeadIO.optimizeBusUtilization();
    flywheelFollowIO.optimizeBusUtilization();
    hoodIO.optimizeBusUtilization();
    kickerIO.optimizeBusUtilization();

    flywheelFollowIO.setControl(new Follower(RobotConfig2026.CanIDs.SHOOTER_LEAD_CAN_ID, MotorAlignmentValue.Opposed));

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
    inputs.fuelInUpperKicker  = !kickerBeamBreak.get();

    inputs.tempsCelcius[0] = flywheelLeadTempCelsius.getValue().in(Units.Celsius);
    inputs.tempsCelcius[1] = flywheelFollowTempCelsius.getValue().in(Units.Celsius);
    inputs.tempsCelcius[2] = hoodTempCelsius.getValue().in(Units.Celsius);
    inputs.tempsCelcius[3] = kickerTempCelsius.getValue().in(Units.Celsius);
  }

  // HOOD

  @Override
  public void setHoodPosition(Rotation2d rot) {
    hood.setNeutralMode(NeutralModeValue.Brake);
    hood.io.setControl(hoodClosedLoopControl.withPosition(rot.getRotations()));
  }

  @Override
  public void resetHoodSensorPosition(Rotation2d angle) {
    hood.io.setPosition(angle.getRotations());
  }

  // Flywheel

  @Override
  public void setFlywheelRPM(double revPerMin) {
    flywheelLead.io.setControl(flywheelClosedLoop.withVelocity(revPerMin / 60));
  }

  @Override
  public void setHoodClosedLoopConstants(double kP, double kD, double kG, double maxProfiledVelocity,
      double maxProfiledAcceleration) {
    hood.slot0Configs.change((pidConfig) -> {
      pidConfig.kP = kP;
      pidConfig.kD = kD;
      pidConfig.kG = kG;
    });

    hood.motionMagicConfigs.change((mmConfig) -> {
      mmConfig.MotionMagicAcceleration   = maxProfiledAcceleration;
      mmConfig.MotionMagicCruiseVelocity = maxProfiledVelocity;
    });
  }

  @Override
  public void setFlywheelClosedLoopConstants(Shooter.FlywheelTunables tunables) {
    Consumer<Slot0Configs> slot0ConfigsSetter = (pidConfig) -> {
      pidConfig.kP = tunables.KP.get();
      pidConfig.kI = tunables.KI.get();
      pidConfig.kD = tunables.KD.get();
      pidConfig.kS = tunables.KS.get();
      pidConfig.kV = tunables.KV.get();
      pidConfig.kA = 0;
      pidConfig.kG = 0;
    };

    Consumer<MotionMagicConfigs> motionMagicConfigsConsumerSetter = (mmConfig) -> {
      mmConfig.MotionMagicAcceleration = tunables.closedLoopMaxAccelerationConstraint.get();
    };

    flywheelLead.slot0Configs.change(slot0ConfigsSetter);
    flywheelFollow.slot0Configs.change(slot0ConfigsSetter);

    flywheelLead.motionMagicConfigs.change(motionMagicConfigsConsumerSetter);
    flywheelFollow.motionMagicConfigs.change(motionMagicConfigsConsumerSetter);
  }

  @Override
  public void setNeutralMode(NeutralModeValue value) {
    hood.setNeutralMode(value);
  }

  @Override
  public void setKickerClosedLoopConstants(Shooter.KickerTunables tunables) {
    kicker.slot0Configs.change((pidConfig) -> {
      pidConfig.kP = tunables.KP.get();
      pidConfig.kI = tunables.KI.get();
      pidConfig.kD = tunables.KD.get();
      pidConfig.kS = tunables.KS.get();
      pidConfig.kV = tunables.KV.get();
      pidConfig.kA = 0;
      pidConfig.kG = 0;
    });

    kicker.motionMagicConfigs.change((mmConfig) -> {
      mmConfig.MotionMagicAcceleration = tunables.closedLoopMaxAccelerationConstraint.get();
    });
  }

  @Override
  public void setKickerVelocity(double revPerMin) {
    kickerTargetRPM = revPerMin;
    kicker.io.setControl(kickerClosedLoop.withVelocity(revPerMin / 60.0));
  }
}
