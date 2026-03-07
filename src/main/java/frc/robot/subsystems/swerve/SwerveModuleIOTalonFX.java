// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.configs.IndividualSwerveModuleConfig;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.TalonFXWrapper;

import java.util.List;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn
 * motor controller, and CANcoder
 *
 * <p>
 * NOTE: This implementation should be used as a starting point and adapted to
 * different hardware configurations (e.g. If using an analog encoder, copy from
 * "ModuleIOSparkMax")
 *
 * <p>
 * To calibrate the absolute encoder offsets, point the modules straight (such
 * that forward motion on the drive motor will propel the robot forward) and
 * copy the reported values from the absolute encoders using AdvantageScope.
 * These values are logged under "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class SwerveModuleIOTalonFX implements SwerveModuleIO {
  private final InvertedValue driveMotorInverted;
  private final InvertedValue steerMotorInverted;

  private final Rotation2d absoluteEncoderOffset;

  private final double DRIVE_GEAR_RATIO;
  private final double TURN_GEAR_RATIO;

  private final double distanceToRotation;

  // Motors and encoder
  private final TalonFXWrapper driveTalon;
  private final TalonFXWrapper turnTalon;
  private final CANcoder       cancoder;

  // Status signals
  private final StatusSignal<Angle>           drivePosition;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage>         driveAppliedVolts;
  private final StatusSignal<Current>         driveCurrent;

  private final StatusSignal<Angle>           turnAbsolutePosition;
  private final StatusSignal<Angle>           turnPosition;
  private final StatusSignal<AngularVelocity> turnVelocity;
  private final StatusSignal<Voltage>         turnAppliedVolts;
  private final StatusSignal<Current>         turnCurrent;

  private final VoltageOut driveVoltageRequest;
  private final VoltageOut steerVoltageRequest;

  private final MotionMagicVelocityVoltage driveMotionMagicVelocityRequest = new MotionMagicVelocityVoltage(0.0)
      .withAcceleration(0.0).withEnableFOC(true);

  private final PositionVoltage turnMotionMagicVoltageRequest = new PositionVoltage(0.0).withEnableFOC(true);

  private Rotation2d turnRelativeOffset; // Relative + Offset = Absolute
  private double     lastPositionMeters;

  private final BaseStatusSignal[] refreshSet;

  public SwerveModuleIOTalonFX(RobotConfig globalConfig, IndividualSwerveModuleConfig moduleSpecificConfig) {
    // --- define constants ---

    this.distanceToRotation = 1.0 / globalConfig.getWheelRadius().in(edu.wpi.first.units.Units.Meters);

    driveMotorInverted = moduleSpecificConfig.driveMotorInverted();
    steerMotorInverted = moduleSpecificConfig.steerMotorInverted();

    absoluteEncoderOffset = moduleSpecificConfig.absoluteEncoderOffset();

    DRIVE_GEAR_RATIO = globalConfig.getSwerveModuleDriveGearRatio();
    TURN_GEAR_RATIO  = globalConfig.getSwerveModuleTurnGearRatio();

    driveTalon = globalConfig.newMotor(moduleSpecificConfig.driveMotorCANID(), RobotConfig.CANSpeed.CANIVORE);
    turnTalon  = globalConfig.newMotor(moduleSpecificConfig.steerMotorCANID(), RobotConfig.CANSpeed.CANIVORE);
    cancoder   = globalConfig.newCancoder(moduleSpecificConfig.steerEncoderCANID(), RobotConfig.CANSpeed.CANIVORE);

    // -- DRIVE
    driveTalon.talonFXConfiguration.change((driveTalonConfig) -> {
      var currentLimitConfig = new CurrentLimitsConfigs();
      currentLimitConfig.StatorCurrentLimit       = 100;
      currentLimitConfig.SupplyCurrentLimit       = 60;
      currentLimitConfig.StatorCurrentLimitEnable = true;

      driveTalonConfig.CurrentLimits                     = currentLimitConfig;
      driveTalonConfig.MotorOutput.Inverted              = driveMotorInverted;
      driveTalonConfig.Voltage.SupplyVoltageTimeConstant = 0.02;

      driveTalonConfig.Feedback.SensorToMechanismRatio = DRIVE_GEAR_RATIO;
    });

    setDriveBrakeMode(true);

    // -- STEER
    turnTalon.talonFXConfiguration.change((turnTalonConfig) -> {
      turnTalonConfig.CurrentLimits        = globalConfig.getSteerTalonCurrentLimitConfig();
      turnTalonConfig.MotorOutput.Inverted = steerMotorInverted;
      // turnTalonConfig.Voltage.SupplyVoltageTimeConstant = 0.02;

      turnTalonConfig.ClosedLoopGeneral.ContinuousWrap = true;
      turnTalonConfig.Feedback.SensorToMechanismRatio  = TURN_GEAR_RATIO;
    });

    setTurnBrakeMode(true);

    // -- CANCODER
    var cancoderConfiguration = new CANcoderConfiguration();
    if (moduleSpecificConfig.encoderInverted() == InvertedValue.CounterClockwise_Positive) {
      var magnetSensor = new MagnetSensorConfigs().withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);
      cancoderConfiguration.withMagnetSensor(magnetSensor);
    }
    cancoder.getConfigurator().apply(cancoderConfiguration);

    // -- configure voltage requests
    driveVoltageRequest           = new VoltageOut(0.0);
    driveVoltageRequest.EnableFOC = globalConfig.getPhoenix6Licensed();

    steerVoltageRequest           = new VoltageOut(0.0);
    steerVoltageRequest.EnableFOC = globalConfig.getPhoenix6Licensed();

    drivePosition     = driveTalon.io.getPosition();
    driveVelocity     = driveTalon.io.getVelocity();
    driveAppliedVolts = driveTalon.io.getMotorVoltage();
    driveCurrent      = driveTalon.io.getStatorCurrent();

    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition         = turnTalon.io.getPosition();
    turnVelocity         = turnTalon.io.getVelocity();
    turnAppliedVolts     = turnTalon.io.getMotorVoltage();
    turnCurrent          = turnTalon.io.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(SwerveModule.ODOMETRY_FREQUENCY, drivePosition, turnPosition,
        turnAbsolutePosition);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, driveVelocity, driveAppliedVolts, driveCurrent, turnVelocity,
        turnAppliedVolts, turnCurrent);
    driveTalon.io.optimizeBusUtilization();
    turnTalon.io.optimizeBusUtilization();
    cancoder.optimizeBusUtilization();

    refreshSet = new BaseStatusSignal[] {
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent
    };
  }

  @Override
  public void updateInputs(Inputs inputs) {
    inputs.refreshAll(refreshSet);

    inputs.driveVelocityRadPerSec = driveVelocity.getValue().in(Units.RadiansPerSecond);
    inputs.driveAppliedVolts      = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps       = driveCurrent.getValueAsDouble();

    inputs.turnVelocityRadPerSec = turnVelocity.getValue().in(Units.RadiansPerSecond);
    inputs.turnAppliedVolts      = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps       = turnCurrent.getValueAsDouble();
  }

  @Override
  public void registerSignalForOdometry(List<BaseStatusSignal> signals) {
    signals.add(drivePosition);
    signals.add(turnPosition);
    signals.add(turnAbsolutePosition);
  }

  @Override
  public SwerveModulePosition updateOdometry(Inputs inputs, double wheelRadius) {
    // Process drive motor position.
    var drivePositionRaw = drivePosition.getValue();
    inputs.drivePositionRad = drivePositionRaw.in(Units.Radians);

    // Process turn motor position.
    var turnPositionAngle = turnPosition.getValue().in(Units.Rotations);
    var angleRelative     = Rotation2d.fromRotations(turnPositionAngle);
    inputs.turnPosition = angleRelative;

    // Process turn encoder position.
    var turnAbsolutePositionRotations = turnAbsolutePosition.getValue().in(Units.Rotations);
    inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePositionRotations).minus(absoluteEncoderOffset);

    // On first cycle, reset relative turn encoder
    // Wait until absolute angle is nonzero in case it wasn't initialized yet
    if (turnRelativeOffset == null && turnAbsolutePositionRotations == 0.0) {
      return null;
    }

    if (DriverStation.isDisabled()) {
      turnRelativeOffset = inputs.turnAbsolutePosition.minus(inputs.turnPosition);
    }

    inputs.angle = angleRelative.plus(turnRelativeOffset);

    var positionMeters = drivePositionRaw.in(Units.Radians) * wheelRadius;
    var res            = new SwerveModulePosition(positionMeters - lastPositionMeters, inputs.angle);
    lastPositionMeters = positionMeters;

    return res;
  }

  // Drive

  @Override
  public void setDriveVoltage(double volts) {
    driveTalon.io.setControl(driveVoltageRequest.withOutput(volts));
  }

  @Override
  public void setDriveVelocity(double velocityMetersPerSec, double accelerationMetersPerSecondSquared) {

    // m/s -> divide by wheel radius to get radians/s -> convert to rotations
    var velocityRotationsPerSecond            = Units.Radians.of(velocityMetersPerSec).in(Units.Rotations)
        * distanceToRotation;
    var accelerationRotationsPerSecondSquared = Units.Radians.of(accelerationMetersPerSecondSquared).in(Units.Rotations)
        * distanceToRotation;

    driveTalon.io.setControl(driveMotionMagicVelocityRequest.withVelocity(velocityRotationsPerSecond)
        .withAcceleration(accelerationRotationsPerSecondSquared));
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveTalon.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setDriveClosedLoopConstraints(double kP, double kD, double kS, double kV, double kA) {
    driveTalon.slot0Configs.change((pidConfig) -> {
      pidConfig.kP = kP;
      pidConfig.kD = kD;
      pidConfig.kS = kS;
      pidConfig.kV = kV;
      pidConfig.kA = kA;
    });
  }

  // Turn

  @Override
  public void setTurnVoltage(double volts) {
    turnTalon.io.setControl(steerVoltageRequest.withOutput(volts));
  }

  @Override
  public void setTurnPosition(Rotation2d position) {
    if (turnRelativeOffset != null) {
      position = position.minus(turnRelativeOffset);
    }

    turnTalon.io.setControl(turnMotionMagicVoltageRequest.withPosition(position.getRotations()));
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnTalon.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setTurnClosedLoopConstraints(double kP, double kD, double cruiseVelocity, double acceleration) {
    turnTalon.slot0Configs.change((pidConfig) -> {
      pidConfig.kP = kP;
      pidConfig.kD = kD;
    });

    turnTalon.motionMagicConfigs.change((mmConfig) -> {
      mmConfig.MotionMagicCruiseVelocity = cruiseVelocity;
      mmConfig.MotionMagicAcceleration   = acceleration;
    });
  }
}
