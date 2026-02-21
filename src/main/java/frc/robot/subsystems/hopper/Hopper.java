package frc.robot.subsystems.hopper;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.*;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.RobotMode.RobotType;
import frc.robot.Constants.ShooterConstants;

public class Hopper extends SubsystemBase {
  public final String ROOT_TABLE = "Hopper";

  public enum Goal {
    Idle, Override, Shooting, PreLoading, Unjamming
  }

  public static class Logs extends AutoRegisterLoggers {
    public LoggerEntry.Decimal rollerVelocityRPM;
    public LoggerEntry.Decimal rollerCurrentAmps;
    public LoggerEntry.Decimal rollerTempCelsius;
    public LoggerEntry.Decimal rollerAppliedVolts;
    public LoggerEntry.Decimal rollerTargetVelocityRPM;

    public LoggerEntry.EnumValue<Goal> hopperState;
  }

  public final Logs logs = LoggerGroup.root.newAutoLogger(Logs.class, ROOT_TABLE);

  // Execution timing
  private final ExecutionTiming timing = new ExecutionTiming(ROOT_TABLE);

  public static class Tunables extends AutoRegisterTunables {
    public LoggedTunableNumber KS;
    public LoggedTunableNumber KP;
    public LoggedTunableNumber KV;
    public LoggedTunableNumber rollerTargetAccelerationConfig;
  }

  private final Tunables tunables = TunableNumberGroup.newAutoTunable(Tunables.class, ROOT_TABLE);

  private final HopperIO        io;
  private final HopperIO.Inputs inputs = new HopperIO.Inputs(logs.group);

  private Goal goal = Goal.Idle;

  private double rollerTargetRPM;

  private Trigger stallDetected = new Trigger(() -> inputs.rollerVelocityRPM <= 50 && inputs.rollerAppliedVolts >= 0.5)
      .debounce(.1);

  public Hopper(HopperIO io) {
    this.io = io;

    if (Constants.RobotMode.getRobot() == RobotType.ROBOT_2025_RETIRED_HOLO) {
      tunables.KS.initDefault(0);
      tunables.KP.initDefault(0.4);
      tunables.KV.initDefault(0.13);
      tunables.rollerTargetAccelerationConfig.initDefault(200);

    } else if (Constants.RobotMode.isSimBot()) {
      tunables.KS.initDefault(0);
      tunables.KP.initDefault(0.00006);
      tunables.KV.initDefault(0.0002);
      tunables.rollerTargetAccelerationConfig.initDefault(0.0);

    } else {
      tunables.KS.initDefault(0);
      tunables.KP.initDefault(0.4);
      tunables.KV.initDefault(0.13);
      tunables.rollerTargetAccelerationConfig.initDefault(200);
    }

    setRollerConstants();
  }

  @Override
  public void periodic() {
    try (var ignored = timing.start()) {
      // Logging
      io.updateInputs(inputs);
      logs.rollerVelocityRPM.info(inputs.rollerVelocityRPM);
      logs.rollerCurrentAmps.info(inputs.rollerCurrentAmps);
      logs.rollerTempCelsius.info(inputs.rollerTempCelsius);
      logs.rollerAppliedVolts.info(inputs.rollerAppliedVolts);

      logs.hopperState.info(goal);

      if (tunables.KS.hasChanged() || tunables.KP.hasChanged() || tunables.KV.hasChanged()
          || tunables.rollerTargetAccelerationConfig.hasChanged()) {
        setRollerConstants();
      }

      switch (goal) {
        case Override:
          break;

        case Idle:
          setRollerVelocity(0);
          break;

        case PreLoading:
        case Shooting:
          setRollerVelocity(HopperConstants.RollerConstants.SHOOTING_ROLLER_VELOCITY);
          break;
        case Unjamming:
          setRollerVelocity(-(HopperConstants.RollerConstants.SHOOTING_ROLLER_VELOCITY));
          break;
        default:
          break;
      }
    }
  }

  // Setters

  private void setRollerConstants() {
    io.setRollerClosedLoopConstants(tunables.KP.get(), tunables.KV.get(), tunables.KS.get(),
        tunables.rollerTargetAccelerationConfig.get());
  }

  public void setRollerVelocity(double revPerMin) {
    io.setRollerVelocity(revPerMin);
    rollerTargetRPM = revPerMin;
    logs.rollerTargetVelocityRPM.info(rollerTargetRPM);
  }

  public void setState(Goal goal) {
    this.goal = goal;
  }

  // Getters

  public Current getRollerCurrentDraw() {
    return Units.Amps.of(inputs.rollerCurrentAmps);
  }

  public AngularVelocity getRollerVelocity() {
    return Units.RPM.of(inputs.rollerVelocityRPM);
  }

  public HopperIOSim getSim() {
    if (io instanceof HopperIOSim) {
      return (HopperIOSim) io;
    }

    return null;
  }

  public boolean rollerStallDetected() {
    return stallDetected.getAsBoolean();
  }

  public void setGoal(Goal goal) {
    this.goal = goal;
  }
}
