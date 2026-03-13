// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.AutoRegisterLoggers;
import frc.lib.team2930.GeometryUtil;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.DrivetrainWrapper;
import java.util.function.DoubleSupplier;

public class DrivetrainDefaultTeleopDrive extends Command {
  private static final String ROOT_TABLE = "TeleopDrive";

  public static class Logs extends AutoRegisterLoggers {
    public LoggerEntry.Decimal               xSupplier;
    public LoggerEntry.Decimal               ySupplier;
    public LoggerEntry.Decimal               omegaSupplier;
    public LoggerEntry.Decimal               linearMagnitude;
    public LoggerEntry.Decimal               linearDirection;
    public LoggerEntry.Struct<Translation2d> linearVelocity;
  }

  public static final Logs logs = LoggerGroup.root.newAutoLogger(Logs.class, ROOT_TABLE);

  /** Creates a new DrivetrainDefaultTeleopDrive. */
  private final DrivetrainWrapper drivetrain;

  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier omegaSupplier;

  private double DEADBAND = 0.1;

  public DrivetrainDefaultTeleopDrive(DrivetrainWrapper drivetrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {

    this.drivetrain = drivetrain;

    this.xSupplier     = xSupplier;
    this.ySupplier     = ySupplier;
    this.omegaSupplier = omegaSupplier;

    addRequirements(drivetrain.getRequirements());
    setName("DrivetrainDefaultTeleopDrive");
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double     redFlip         = (Constants.isRedAlliance() ? -1.0 : 1.0);
    double     x               = xSupplier.getAsDouble() * redFlip;
    double     y               = ySupplier.getAsDouble() * redFlip;
    double     linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = GeometryUtil.newRotation2d(x, y);
    double     omega           = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

    // Square values
    linearMagnitude = linearMagnitude * linearMagnitude;
    omega           = Math.copySign(omega * omega, omega);

    // Calcaulate new linear velocity
    Translation2d linearVelocity = new Pose2d(Constants.zeroTranslation2d, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Constants.zeroRotation2d)).getTranslation();

    logs.xSupplier.info(xSupplier.getAsDouble());
    logs.ySupplier.info(ySupplier.getAsDouble());
    logs.omegaSupplier.info(omegaSupplier.getAsDouble());

    logs.linearMagnitude.info(linearMagnitude);
    logs.linearDirection.info(linearDirection);
    logs.linearVelocity.info(linearVelocity);

    // Convert to field relative speeds & send command
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
        linearVelocity.getX() * drivetrain.getMaxLinearSpeed().in(Units.MetersPerSecond),
        linearVelocity.getY() * drivetrain.getMaxLinearSpeed().in(Units.MetersPerSecond),
        omega * drivetrain.getMaxAngularSpeed().in(Units.RadiansPerSecond));

    drivetrain.setVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, drivetrain.getRotationGyroOnly()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
