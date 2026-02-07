package frc.robot.subsystems.hopper;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.ExecutionTiming;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.RobotMode.RobotType;

public class Hopper extends SubsystemBase {

  public static final String ROOT_TABLE = "Hopper";

  public enum HopperState {
    Idle, Override, Shooting
  }

  // Execution timing
  private static final ExecutionTiming timing = new ExecutionTiming(ROOT_TABLE);

  // Logging
  private static final LoggerGroup logGroup = LoggerGroup.build(ROOT_TABLE);

  // Rollers
  private static final LoggerEntry.Decimal logInputs_rollerVelocityRPM  = logGroup.buildDecimal("VelocityRPM");
  private static final LoggerEntry.Decimal logInputs_rollerCurrentAmps  = logGroup.buildDecimal("CurrentAmps");
  private static final LoggerEntry.Decimal logInputs_rollerTempCelsius  = logGroup.buildDecimal("TempCelsius");
  private static final LoggerEntry.Decimal logInputs_rollerAppliedVolts = logGroup.buildDecimal("AppliedVolts");

  private static final LoggerEntry.Decimal logRollerTargetVelocityRPM = logGroup.buildDecimal("TargetVelocityRPM");

  private static final LoggerEntry.EnumValue<HopperState> logHopperState = logGroup.buildEnum("HopperState");

  private static final TunableNumberGroup group = new TunableNumberGroup(HopperConstants.RollerConstants.ROLLER);
  // Roller
  private static final LoggedTunableNumber rKS                            = group.build("KS");
  private static final LoggedTunableNumber rKP                            = group.build("KP");
  private static final LoggedTunableNumber rKV                            = group.build("KV");
  private static final LoggedTunableNumber rollerTargetAccelerationConfig = group.build("MaxAccelerationConstraint");

  static {
    if (Constants.RobotMode.getRobot() == RobotType.ROBOT_2025_RETIRED_HOLO) {
      rKS.initDefault(0);
      rKP.initDefault(0.4);
      rKV.initDefault(0.13);
      rollerTargetAccelerationConfig.initDefault(200);

    } else if (Constants.RobotMode.isSimBot()) {
      rKS.initDefault(0);
      rKP.initDefault(0.00006);
      rKV.initDefault(0.0002);
      rollerTargetAccelerationConfig.initDefault(0.0);

    } else {
      rKS.initDefault(0);
      rKP.initDefault(0.4);
      rKV.initDefault(0.13);
      rollerTargetAccelerationConfig.initDefault(200);
    }
  }

  private final HopperIO        io;
  private final HopperIO.Inputs inputs = new HopperIO.Inputs(logGroup);

  private HopperState hopperState = HopperState.Idle;

  private double rollerTargetRPM;

  private Trigger stallDetected = new Trigger(() -> inputs.rollerVelocityRPM <= 50 && inputs.rollerAppliedVolts >= 0.5)
      .debounce(.1);

  public Hopper(HopperIO io) {
    this.io = io;

    setRollerConstants();
  }

  @Override
  public void periodic() {
    try (var ignored = timing.start()) {
      // Logging
      io.updateInputs(inputs);
      logInputs_rollerVelocityRPM.info(inputs.rollerVelocityRPM);
      logInputs_rollerCurrentAmps.info(inputs.rollerCurrentAmps);
      logInputs_rollerTempCelsius.info(inputs.rollerTempCelsius);
      logInputs_rollerAppliedVolts.info(inputs.rollerAppliedVolts);

      logHopperState.info(hopperState);

      var rhc = hashCode();
      if (rKS.hasChanged(rhc) || rKP.hasChanged(rhc) || rKV.hasChanged(rhc)
          || rollerTargetAccelerationConfig.hasChanged(rhc)) {
        setRollerConstants();
      }

      switch (hopperState) {
        case Override:
          break;

        case Idle:
          setRollerVelocity(0);
          break;

        case Shooting:
          setRollerVelocity(HopperConstants.RollerConstants.SHOOTING_ROLLER_VELOCITY);
          break;

        default:
          break;
      }
    }
  }

  // Setters

  private void setRollerConstants() {
    io.setRollerClosedLoopConstants(rKP.get(), rKV.get(), rKS.get(), rollerTargetAccelerationConfig.get());
  }

  public void setRollerVelocity(double revPerMin) {
    io.setRollerVelocity(revPerMin);
    rollerTargetRPM = revPerMin;
    logRollerTargetVelocityRPM.info(rollerTargetRPM);
  }

  public void setState(HopperState state) {
    hopperState = state;
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
}
