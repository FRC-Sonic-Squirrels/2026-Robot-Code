package frc.robot.subsystems.climber;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.*;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode.RobotType;

public class Elevator extends SubsystemBase {

  public static class Logs extends AutoRegisterLoggers {
    public LoggerEntry.Decimal targetHeightInches;
    public LoggerEntry.Decimal heightInches;
    public LoggerEntry.Decimal velocityInchesPerSecond;
    public LoggerEntry.Decimal appliedVolts;
    public LoggerEntry.Decimal currentAmps;
    public LoggerEntry.Decimal tempCelsius;
  }

  public final Logs logs;

  // Execution timing
  private final ExecutionTiming timing;

  public static class Tunables extends AutoRegisterTunables {
    public LoggedTunableNumber elevatorKP;
    public LoggedTunableNumber elevatorKD;
    public LoggedTunableNumber elevatorKG;

    public LoggedTunableNumber maxVelocityConfig;
    public LoggedTunableNumber targetAccelerationConfig;

    @TunableEntry(defaultValue = 1)
    public LoggedTunableNumber toleranceHeight;

    public boolean hasChanged() {
      return elevatorKP.hasChanged() || elevatorKD.hasChanged() || elevatorKG.hasChanged();
    }
  }

  private final Tunables tunables;

  private double setAccel;

  private final ElevatorIO        io;
  private final ElevatorIO.Inputs inputs;

  private Distance    targetHeight = Units.Meters.zero();
  private ControlMode controlMode  = ControlMode.OPEN_LOOP;

  /** Creates a new ElevatorSubsystem. */
  public Elevator(LoggerGroup group, ElevatorIO io) {
    this.io       = io;
    this.timing   = new ExecutionTiming(group.toString());
    this.logs     = LoggerGroup.root.newAutoLogger(Logs.class, group.toString());
    this.inputs   = new ElevatorIO.Inputs(logs.group);
    this.tunables = TunableNumberGroup.newAutoTunable(Tunables.class, group.toString());

    setAccel = tunables.targetAccelerationConfig.get();

    if (Constants.RobotMode.getRobot() == RobotType.ROBOT_2025_RETIRED_HOLO) {
      tunables.elevatorKP.initDefault(4);
      tunables.elevatorKD.initDefault(0);
      tunables.elevatorKG.initDefault(0.0);

      tunables.maxVelocityConfig.initDefault(10);
      tunables.targetAccelerationConfig.initDefault(10);

    } else if (Constants.RobotMode.getRobot() == RobotType.ROBOT_SIMBOT) {
      tunables.elevatorKP.initDefault(.2);
      tunables.elevatorKD.initDefault(0);
      tunables.elevatorKG.initDefault(0.0);

      tunables.maxVelocityConfig.initDefault(40);
      tunables.targetAccelerationConfig.initDefault(80);
    } else {
      tunables.elevatorKP.initDefault(4);
      tunables.elevatorKD.initDefault(0);
      tunables.elevatorKG.initDefault(0.0);
    }

    setConstants();
  }

  @Override
  public void periodic() {
    try (var ignored = timing.start()) {
      // Elevator logging
      io.updateInputs(inputs);

      logs.heightInches.info(inputs.heightInches);
      logs.velocityInchesPerSecond.info(inputs.velocityInchesPerSecond);
      logs.appliedVolts.info(inputs.appliedVolts);
      logs.currentAmps.info(inputs.currentAmps);
      logs.tempCelsius.info(inputs.tempCelsius);

      // Updating tunable numbers
      if (tunables.hasChanged()) {
        setConstants();
      }
    }
  }

  // Setters

  public void setPercentOut(double percent) {
    io.setVoltage(percent);
    controlMode = ControlMode.OPEN_LOOP;
  }

  public void setHeight(Distance height) {
    setHeight(height, tunables.targetAccelerationConfig.get());
  }

  /**
   * Sets height and elevator acceleration
   *
   * @param height
   * @param accel
   */
  public void setHeight(Distance height, double accel) {

    if (accel != setAccel) {
      setAccel = accel;

      double kP             = tunables.elevatorKP.get();
      double kD             = tunables.elevatorKD.get();
      double kG             = tunables.elevatorKG.get();
      double cruiseVelocity = tunables.maxVelocityConfig.get();
      io.setClosedLoopConstants(kP, kD, kG, accel, cruiseVelocity);
    }

    io.setHeight(height);
    targetHeight = height;
    logs.targetHeightInches.info(targetHeight.in(Units.Inches));
    controlMode = ControlMode.CLOSED_LOOP;
  }

  public void resetSensorToHomePosition() {
    io.setSensorPosition(Constants.ClimberConstants.LEADER_HEIGHT_STOWED);
  }

  public void setNeutralMode(NeutralModeValue value) {
    io.setNeutralMode(value);
  }

  private void setConstants() {
    double kP = tunables.elevatorKP.get();
    double kD = tunables.elevatorKD.get();
    double kG = tunables.elevatorKG.get();

    var acceleration   = tunables.targetAccelerationConfig.get();
    var cruiseVelocity = tunables.maxVelocityConfig.get();
    io.setClosedLoopConstants(kP, kD, kG, acceleration, cruiseVelocity);
    setAccel = acceleration;
  }

  public void setElevatorManualControl(double percent) {
    setPercentOut(2 * percent + tunables.elevatorKG.get());
  }

  // Getters

  public boolean isAtTarget() {
    return Math.abs(targetHeight.in(Units.Inches) - inputs.heightInches) <= tunables.toleranceHeight.get();
  }

  public boolean isAtTarget(Distance height) {
    return Math.abs(height.in(Units.Inches) - inputs.heightInches) <= tunables.toleranceHeight.get();
  }

  public Distance getHeight() {
    return Units.Inches.of(inputs.heightInches);
  }

  public Voltage getVoltage() {
    return Units.Volts.of(inputs.appliedVolts);
  }

  public LinearVelocity getVelocity() {
    return Units.InchesPerSecond.of(inputs.velocityInchesPerSecond);
  }
}
