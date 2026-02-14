package frc.robot.subsystems.climber;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.*;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode.RobotType;

public class Elevator extends SubsystemBase {
  private final String ROOT_TABLE = "Elevator";

  public static class ElevatorLogs extends AutoRegisterLoggers {
    public LoggerEntry.Decimal appliedVolts;
    public LoggerEntry.Decimal currentAmps;
    public LoggerEntry.Decimal tempCelsius;
    public LoggerEntry.Decimal velocityDegreesPerSecond;
    public LoggerEntry.Decimal targetHeightInches;
  }

  public static class Logs extends AutoRegisterLoggers {
    public ElevatorLogs elevator;
  }

  public final Logs logs = LoggerGroup.root.newAutoLogger(Logs.class, ROOT_TABLE);

  // Execution timing
  private final ExecutionTiming elevatorTiming = new ExecutionTiming(ROOT_TABLE);

  public static class Tunables extends AutoRegisterTunables {
    public LoggedTunableNumber elevatorKP;
    public LoggedTunableNumber elevatorKD;
    public LoggedTunableNumber elevatorKG;

    public LoggedTunableNumber maxVelocityConfig;
    public LoggedTunableNumber elevatorTargetAccelerationConfig;

    @TunableEntry(defaultValue = 40.0)
    public LoggedTunableNumber toleranceDegrees;

    @TunableEntry(defaultValue = 1.0)
    public LoggedTunableNumber toleranceHeight;

    public boolean hasChanged() {
      return elevatorKP.hasChanged() || elevatorKD.hasChanged() || elevatorKG.hasChanged();

    }
  }

  private final Tunables tunables = TunableNumberGroup.newAutoTunable(Tunables.class, ROOT_TABLE);

  private final ElevatorIO        io;
  private final ElevatorIO.Inputs elevatorInputs;

  private double elevatorHeight;

  /** Creates a new ElevatorSubsystem. */
  public Elevator(ElevatorIO io) {
    this.io             = io;
    this.elevatorInputs = new ElevatorIO.Inputs(logs.group);

    if (Constants.RobotMode.getRobot() == RobotType.ROBOT_2025_RETIRED_HOLO) {
      tunables.elevatorKP.initDefault(4);
      tunables.elevatorKD.initDefault(0);
      tunables.elevatorKG.initDefault(0.0);

      tunables.maxVelocityConfig.initDefault(10);
      tunables.elevatorTargetAccelerationConfig.initDefault(10);

    } else if (Constants.RobotMode.getRobot() == RobotType.ROBOT_SIMBOT) {
      tunables.elevatorKP.initDefault(.02);
      tunables.elevatorKD.initDefault(0);
      tunables.elevatorKG.initDefault(0.0);

      tunables.maxVelocityConfig.initDefault(40);
      tunables.elevatorTargetAccelerationConfig.initDefault(80);
    } else {
      tunables.elevatorKP.initDefault(4);
      tunables.elevatorKD.initDefault(0);
      tunables.elevatorKG.initDefault(0.0);
    }

    setConstants();
  }

  @Override
  public void periodic() {
    try (var ignored = elevatorTiming.start()) {
      // Elevator logging
      io.updateElevatorInputs(elevatorInputs);

      logs.elevator.appliedVolts.info(elevatorInputs.appliedVolts);
      logs.elevator.currentAmps.info(elevatorInputs.currentAmps);
      logs.elevator.tempCelsius.info(elevatorInputs.tempCelsius);
      logs.elevator.velocityDegreesPerSecond.info(elevatorInputs.velocityDegreesPerSecond);
      logs.elevator.targetHeightInches.info(elevatorHeight);

      // Updating tunable numbers
      if (tunables.hasChanged()) {
        setConstants();
      }
    }
  }

  // setters

  private void setConstants() {
    double kP = tunables.elevatorKP.get();
    double kD = tunables.elevatorKD.get();
    double kG = tunables.elevatorKG.get();
    io.setElevatorClosedLoopConstants(kP, kD, kG);
  }

  public void setTargetHeight(double height) {
    io.setElevatorClosedLoopPosition(height);
  }

  public boolean setElevatorNeutralMode(NeutralModeValue value) {
    return io.setElevatorNeutralMode(value);
  }

  // getters

  public boolean isElevatorAtTargetHeight(double target) {
    var error = elevatorInputs.position - target;
    return error <= tunables.toleranceHeight.get();
  }

  public double getElevatorHeight() {
    return elevatorHeight;
  }

  public Voltage getElevatorVoltage() {
    return Units.Volts.of(elevatorInputs.appliedVolts);
  }

  public AngularVelocity getElevatorVelocity() {
    return Units.DegreesPerSecond.of(elevatorInputs.velocityDegreesPerSecond);
  }
}
