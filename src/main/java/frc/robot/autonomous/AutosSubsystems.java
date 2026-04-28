package frc.robot.autonomous;

import frc.robot.subsystems.LED;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.swerve.DrivetrainWrapper;

public record AutosSubsystems(DrivetrainWrapper drivetrain, LED led, Intake leftIntake, Intake rightIntake,
    Turret turret) {
}
