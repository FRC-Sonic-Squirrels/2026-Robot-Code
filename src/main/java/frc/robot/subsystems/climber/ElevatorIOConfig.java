package frc.robot.subsystems.climber;

import com.ctre.phoenix6.hardware.TalonFX;

public record ElevatorIOConfig(TalonFX motor, Boolean motorInverted, double gearRatio, double carriageMass,
    double pulleyDiameter, double maxHeight) {
}
