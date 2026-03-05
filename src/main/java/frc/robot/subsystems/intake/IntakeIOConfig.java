package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

public record IntakeIOConfig(TalonFX pivotMotor, InvertedValue pivotMotorInverted, TalonFX rollerMotor,
    InvertedValue rollerMotorInverted) {
}
