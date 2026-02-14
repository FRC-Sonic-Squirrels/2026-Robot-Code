// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team2930.*;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.configs.RobotConfig;
import frc.robot.subsystems.swerve.DrivetrainWrapper;

/* You should consider using the more terse Command factories API instead https://docs.wpilib
.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WheelRadiusCharacterization extends Command {
  private final String ROOT_TABLE = "WheelRadiusCharacterization";

  /** Creates a new WheelRadiusCharacterization. */
  public static class Logs extends AutoRegisterLoggers {
    public LoggerEntry.Decimal currentEstimatedRadius;
  }

  public final Logs logs = LoggerGroup.root.newAutoLogger(Logs.class, ROOT_TABLE);

  public static class Tunables extends AutoRegisterTunables {
    @TunableEntry(defaultValue = 0.5)
    // radians per second
    public LoggedTunableNumber characterizationSpeed;
  }

  private final Tunables tunables = TunableNumberGroup.newAutoTunable(Tunables.class, ROOT_TABLE);

  private final DrivetrainWrapper drivetrainWrapper;
  // inches
  private final double driveBaseRadius;
  // radians
  private double lastYaw;
  // radians
  private double totalYaw;
  // radians
  private Angle[] initialWheelRotations;

  // rotates the robot in place at characterizationSpeed rads/sec
  // and outputs the radius of the wheels in inches
  public WheelRadiusCharacterization(DrivetrainWrapper drivetrainWrapper, RobotConfig config) {
    this.drivetrainWrapper = drivetrainWrapper;
    driveBaseRadius        = Units.Meters.of(config.getDriveBaseRadius()).in(Units.Inches);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastYaw               = drivetrainWrapper.getHubPoseEstimatorPose(true).getRotation().getRadians();
    totalYaw              = 0.0;
    initialWheelRotations = drivetrainWrapper.getModuleRotations();

    drivetrainWrapper.setRotationOverride(tunables.characterizationSpeed.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    totalYaw += MathUtil
        .angleModulus(drivetrainWrapper.getHubPoseEstimatorPose(true).getRotation().getRadians() - lastYaw);
    lastYaw   = drivetrainWrapper.getHubPoseEstimatorPose(true).getRotation().getRadians();

    double  averageWheelRotation = 0.0;
    Angle[] wheelRotations       = drivetrainWrapper.getModuleRotations();
    for (int i = 0; i < 4; i++) {
      averageWheelRotation += Math
          .abs(wheelRotations[i].in(Units.Radians) - initialWheelRotations[i].in(Units.Radians));
    }
    averageWheelRotation /= 4;
    double currentEstimatedRadius = (totalYaw * driveBaseRadius) / averageWheelRotation;

    logs.currentEstimatedRadius.info(currentEstimatedRadius);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrainWrapper.resetRotationOverride();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // do two rotations before ending
    return totalYaw > Math.PI * 4;
  }
}
