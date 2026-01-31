package frc.robot.autonomous;

import frc.robot.subsystems.LED;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.DrivetrainWrapper;

public record AutosSubsystems(DrivetrainWrapper drivetrain, LED led, Intake intake) {
}
