// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.*;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.subsystems.swerve.DrivetrainWrapper;

import java.util.function.Supplier;

public class RotateToAngle extends Command {
  private final String ROOT_TABLE = "RotateToAngle";

  public static class Logs extends AutoRegisterLoggers {
    public LoggerEntry.Decimal rotVel;
    public LoggerEntry.Decimal targetRotation;
    public LoggerEntry.Decimal currentRotation;
    public LoggerEntry.Decimal positionErrorDeg;
  }

  public final Logs logs = LoggerGroup.root.newAutoLogger(Logs.class, ROOT_TABLE);

  public static class Tunables extends AutoRegisterTunables {
    @TunableEntry(defaultValue = 4.9)
    public LoggedTunableNumber KP;

    @TunableEntry(defaultValue = 2 * Math.PI)
    public LoggedTunableNumber cruiseVelocity;

    @TunableEntry(defaultValue = 4 * Math.PI)
    public LoggedTunableNumber maxAcceleration;

    @TunableEntry(defaultValue = 1)
    public LoggedTunableNumber toleranceDegrees;
  }

  private final Tunables tunables = TunableNumberGroup.newAutoTunable(Tunables.class, ROOT_TABLE);

  private final DrivetrainWrapper     wrapper;
  private final Supplier<Rotation2d>  angle;
  private final Supplier<Pose2d>      currentPose;
  private final ProfiledPIDController rotationalPID = new ProfiledPIDController(0, 0, 0, new Constraints(0.0, 0.0));

  /** Creates a new RotateToAngle. */
  public RotateToAngle(DrivetrainWrapper wrapper, Supplier<Rotation2d> angle, Supplier<Pose2d> pose2d) {
    this.wrapper     = wrapper;
    this.angle       = angle;
    this.currentPose = pose2d;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationalPID.reset(currentPose.get().getRotation().getRadians(),
        wrapper.getFieldRelativeVelocities().getRotation().getRadians());
    rotationalPID.setP(tunables.KP.get());
    rotationalPID.setConstraints(new Constraints(tunables.cruiseVelocity.get(), tunables.maxAcceleration.get()));
    rotationalPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var target              = angle.get();
    var currentPoseRotation = currentPose.get().getRotation();
    var rotVel              = rotationalPID.calculate(currentPoseRotation.getRadians(), target.getRadians());
    wrapper.setRotationOverride(rotVel);

    logs.rotVel.info(rotVel);
    logs.targetRotation.info(target);
    logs.currentRotation.info(currentPoseRotation);
    logs.positionErrorDeg.info(Math.toDegrees(rotationalPID.getPositionError()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrapper.resetRotationOverride();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public boolean withinTolerance() {
    return Math.abs(Math.toDegrees(rotationalPID.getPositionError())) <= tunables.toleranceDegrees.get();
  }

  public double getError() {
    return rotationalPID.getPositionError();
  }
}
