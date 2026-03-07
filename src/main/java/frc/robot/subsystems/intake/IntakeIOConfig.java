package frc.robot.subsystems.intake;

import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.subsystems.TalonFXWrapper;

public record IntakeIOConfig(TalonFXWrapper pivotMotor, InvertedValue pivotMotorInverted, TalonFXWrapper rollerMotor,
    InvertedValue rollerMotorInverted) {
}
