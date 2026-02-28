package frc.robot.subsystems.climber;

import com.ctre.phoenix6.signals.InvertedValue;

public record ElevatorIOConfig(int motorCANID, InvertedValue invertedValue, double gearRatio, double carriageMass,
    double pulleyDiameter, double minHeight, double maxHeight) {

  /**
   * Sensor-to-mechanism ratio derived from gear ratio and pulley diameter.
   * Converts motor rotations to linear inches.
   */
  public double sensorToMechanismRatio() {
    return gearRatio / (pulleyDiameter * Math.PI);
  }
}
