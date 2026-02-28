package frc.robot.subsystems.climber;

public record ElevatorIOConfig(int motorCANID, Boolean motorInverted, double gearRatio, double carriageMass,
    double pulleyDiameter, double maxHeight) {
}
