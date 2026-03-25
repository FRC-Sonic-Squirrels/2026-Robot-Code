package frc.robot.subsystems.intake;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.TalonFXWrapper;

public record IntakeIOConfig(TalonFXWrapper pivotMotor, InvertedValue pivotMotorInverted, TalonFXWrapper rollerMotor,
    InvertedValue rollerMotorInverted, Rotation2d minPivotAngle, Rotation2d maxPivotAngle) {
}
