package frc.robot.subsystems.climber;

import frc.robot.subsystems.TalonFXWrapper;

public record ElevatorIOConfig(TalonFXWrapper motor, Boolean motorInverted, double gearRatio, double carriageMass,
    double pulleyDiameter, double maxHeight) {
}
