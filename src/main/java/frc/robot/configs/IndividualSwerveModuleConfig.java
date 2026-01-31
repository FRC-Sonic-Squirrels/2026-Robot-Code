package frc.robot.configs;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;

public record IndividualSwerveModuleConfig(int driveMotorCANID, int steerMotorCANID, int steerEncoderCANID,
    Rotation2d absoluteEncoderOffset, InvertedValue driveMotorInverted, InvertedValue steerMotorInverted,
    InvertedValue encoderInverted) {

  public static IndividualSwerveModuleConfig configureWithDefaultInverts(int driveMotorCANID, int steerMotorCANID,
      int steerEncoderCANID, Rotation2d absoluteEncoderOffset) {
    return new IndividualSwerveModuleConfig(driveMotorCANID, steerMotorCANID, steerEncoderCANID, absoluteEncoderOffset,
        InvertedValue.CounterClockwise_Positive, InvertedValue.CounterClockwise_Positive,
        InvertedValue.Clockwise_Positive);
  }
}
