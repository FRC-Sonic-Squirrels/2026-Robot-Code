package frc.robot.subsystems.intake;

import com.ctre.phoenix6.signals.InvertedValue;

public record IntakeIOConfig(int pivotMotorCANID, int rollerMotorCANID, InvertedValue pivotMotorInverted,
    InvertedValue rollerMotorInverted) {
}
