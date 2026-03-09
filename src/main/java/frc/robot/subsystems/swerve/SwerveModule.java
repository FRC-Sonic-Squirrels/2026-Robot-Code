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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.team2930.*;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.configs.RobotConfig;

import java.util.List;

public class SwerveModule {
  public static final int ODOMETRY_FREQUENCY = 250;

  public static class Logs extends AutoRegisterLoggers {
    public LoggerEntry.Decimal drivePositionRad;
    public LoggerEntry.Decimal driveVelocityRadPerSec;
    public LoggerEntry.Decimal driveVelocityMetersPerSec;
    public LoggerEntry.Decimal driveTargetVelocityMetersPerSec;
    public LoggerEntry.Decimal driveAppliedVolts;
    public LoggerEntry.Decimal driveCurrentAmps;
    public LoggerEntry.Decimal turnAbsolutePosition;
    public LoggerEntry.Decimal turnPosition;
    public LoggerEntry.Decimal turnVelocityRadPerSec;
    public LoggerEntry.Decimal turnAppliedVolts;
    public LoggerEntry.Decimal turnCurrentAmps;
    public LoggerEntry.Decimal angle;
  }

  public static class Tunables extends AutoRegisterTunables {
    @TunableEntry(defaultValue = 200)
    public LoggedTunableNumber SwerveTurnCruiseVelocity;

    @TunableEntry(defaultValue = 200)
    public LoggedTunableNumber SwerveTurnAcceleration;
  }

  private final Logs            logs;
  private final Tunables        tunables = TunableNumberGroup.newAutoTunable(Tunables.class, "RobotConfig");
  private final ExecutionTiming timing;

  private final SwerveModuleIO        io;
  private final SwerveModuleIO.Inputs inputs;
  private final double                wheelRadius;

  // Tunable numbers
  private final LoggedTunableNumber driveKS;
  private final LoggedTunableNumber driveKV;
  private final LoggedTunableNumber driveKA;

  private final LoggedTunableNumber driveKP;
  private final LoggedTunableNumber driveKD;

  private final LoggedTunableNumber angleKP;
  private final LoggedTunableNumber angleKD;

  private final double maxSpeedMetersPerSecond;

  private Rotation2d angleSetpoint;                    // Setpoint for closed loop control, null for open loop
  private Double     speedSetpoint;                    // Setpoint for closed loop control, null for open loop
  private double     driveMotorMotionMagicAcceleration;

  private boolean motionMagicDrive;

  public SwerveModule(int index, RobotConfig config, SwerveModuleIO io) {
    this.io = io;

    // Logging

    String key = "SwerveModule" + index;

    logs   = LoggerGroup.root.newAutoLogger(Logs.class, key);
    timing = new ExecutionTiming(key);
    inputs = new SwerveModuleIO.Inputs(logs.group);

    wheelRadius = config.getWheelRadius().in(Units.Meters);

    // Tunable numbers

    driveKS = config.getDriveKS();
    driveKV = config.getDriveKV();
    driveKA = config.getDriveKA();

    driveKP = config.getDriveKP();
    driveKD = config.getDriveKD();

    angleKP = config.getAngleKP();
    angleKD = config.getAngleKD();

    maxSpeedMetersPerSecond = config.getRobotMaxLinearVelocity();

    updateConstants();

    setBrakeMode(false);
  }

  public void registerSignalForOdometry(List<BaseStatusSignal> signals) {
    io.registerSignalForOdometry(signals);
  }

  /**
   * Update inputs without running the rest of the periodic logic. This is useful
   * since these updates need to be properly thread-locked.
   */
  public void updateInputs() {
    // Ideally nothing else lives here and is put in periodic() instead
    io.updateInputs(inputs);
  }

  public void periodic() {
    try (var ignored = timing.start()) {
      // Logging
      logs.drivePositionRad.info(inputs.drivePositionRad);
      logs.driveVelocityRadPerSec.info(inputs.driveVelocityRadPerSec);
      logs.driveVelocityMetersPerSec.info(getVelocity().in(Units.MetersPerSecond));
      logs.driveTargetVelocityMetersPerSec.info(speedSetpoint == null ? 0 : speedSetpoint);
      logs.driveAppliedVolts.info(inputs.driveAppliedVolts);
      logs.driveCurrentAmps.info(inputs.driveCurrentAmps);
      logs.turnAbsolutePosition.info(inputs.turnAbsolutePosition);
      logs.turnPosition.info(inputs.turnPosition);
      logs.turnVelocityRadPerSec.info(inputs.turnVelocityRadPerSec);
      logs.turnAppliedVolts.info(inputs.turnAppliedVolts);
      logs.turnCurrentAmps.info(inputs.turnCurrentAmps);
      logs.angle.info(inputs.angle);

      // update Tuneable PID and FF values
      if (driveKS.hasChanged() || driveKV.hasChanged() || driveKA.hasChanged() || driveKP.hasChanged()
          || driveKD.hasChanged() || angleKP.hasChanged() || angleKD.hasChanged()
          || tunables.SwerveTurnCruiseVelocity.hasChanged() || tunables.SwerveTurnAcceleration.hasChanged()) {
        updateConstants();
      }

      // Run closed loop turn control
      if (angleSetpoint != null) {
        io.setTurnPosition(angleSetpoint);
      }

      if (speedSetpoint != null) {
        if (io instanceof SwerveModuleIOSim || motionMagicDrive) {
          io.setDriveVelocity(speedSetpoint, driveMotorMotionMagicAcceleration);
        } else {
          io.setDriveVoltage(speedSetpoint / maxSpeedMetersPerSecond * 12.0);
        }
      }
    }
  }

  private void updateConstants() {
    io.setDriveClosedLoopConstraints(driveKP.get(), driveKD.get(), driveKS.get(), driveKV.get(), driveKA.get());

    double kP             = angleKP.get();
    double kD             = angleKD.get();
    double cruiseVelocity = tunables.SwerveTurnCruiseVelocity.get();
    double acceleration   = tunables.SwerveTurnAcceleration.get();
    io.setTurnClosedLoopConstraints(kP, kD, cruiseVelocity, acceleration);
  }

  public SwerveModulePosition updateOdometry() {
    return io.updateOdometry(inputs, wheelRadius);
  }

  // Setters

  /**
   * Runs the module with the specified setpoint state. Returns the optimized
   * state.
   */
  public SwerveModuleState runSetpoint(SwerveModuleState state, double driveMotorMotionMagicAcceleration,
      boolean motionMagicDrive) {
    // Optimize state based on current angle
    // Controllers run in "periodic" when the setpoint is not null
    state.optimize(getAngle());

    // Update setpoints, controllers run in "periodic"
    angleSetpoint = state.angle;
    speedSetpoint = state.speedMetersPerSecond;

    this.driveMotorMotionMagicAcceleration = driveMotorMotionMagicAcceleration;
    this.motionMagicDrive                  = motionMagicDrive;

    return state;
  }

  /**
   * Runs the module with the specified voltage while controlling to zero degrees.
   */
  public void runCharacterization(double volts) {
    io.setDriveVoltage(volts);
    speedSetpoint = null;
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setTurnVoltage(0.0);
    io.setDriveVoltage(0.0);

    // Disable closed loop control for turn and drive
    angleSetpoint = null;
    speedSetpoint = null;
  }

  /** Sets whether brake mode is enabled. */
  public void setBrakeMode(boolean enabled) {
    io.setDriveBrakeMode(enabled);
    io.setTurnBrakeMode(enabled);
  }

  // Getters

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return inputs.angle;
  }

  /** Returns the current drive position of the module in meters. */
  public Distance getDrivePosition() {
    return Units.Meters.of(inputs.drivePositionRad * wheelRadius);
  }

  public Angle getRawDrivePosition() {
    return Units.Radians.of(inputs.drivePositionRad);
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public LinearVelocity getVelocity() {
    return Units.MetersPerSecond.of(inputs.driveVelocityRadPerSec * wheelRadius);
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDrivePosition(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocity(), getAngle());
  }

  /** Returns the drive velocity in radians/sec. */
  public AngularVelocity getCharacterizationVelocity() {
    return Units.RadiansPerSecond.of(inputs.driveVelocityRadPerSec);
  }

  public Voltage getAppliedVoltage() {
    return Units.Volts.of(inputs.driveAppliedVolts);
  }
}
