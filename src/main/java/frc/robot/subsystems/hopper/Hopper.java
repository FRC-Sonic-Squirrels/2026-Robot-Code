package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.ControlMode;
import frc.lib.team2930.ExecutionTiming;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.lib.team2930.TunableNumberGroup;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.RollerConstants;
import frc.robot.subsystems.intake.Intake.IntakeState;

public class Hopper extends SubsystemBase {

  public enum HopperState {
    Idle,
    Override,
    Shooting
  }

  // Execution timing
  private static final ExecutionTiming timing = new ExecutionTiming(IntakeConstants.ROOT_TABLE);

  // Logging
  private static final LoggerGroup logGroup = LoggerGroup.build(IntakeConstants.ROOT_TABLE);

  // Rollers
  private static final LoggerGroup rollerLogGroup = logGroup.subgroup(RollerConstants.ROOT_TABLE);
  private static final LoggerEntry.Decimal logInputs_rollerVelocityRPM =
      rollerLogGroup.buildDecimal("VelocityRPM");
  private static final LoggerEntry.Decimal logInputs_rollerCurrentAmps =
      rollerLogGroup.buildDecimal("CurrentAmps");
  private static final LoggerEntry.Decimal logInputs_rollerTempCelsius =
      rollerLogGroup.buildDecimal("TempCelsius");
  private static final LoggerEntry.Decimal logInputs_rollerAppliedVolts =
      rollerLogGroup.buildDecimal("AppliedVolts");

  private static final LoggerEntry.Decimal logRollerTargetVelocityRPM =
      rollerLogGroup.buildDecimal("TargetVelocityRPM");
  private static final LoggerEntry.EnumValue<ControlMode> logRollerControlMode =
      rollerLogGroup.buildEnum("ControlMode");

  private static final LoggerEntry.EnumValue<IntakeState> logIntakeState =
      logGroup.buildEnum("IntakeState");

  private static final TunableNumberGroup group =
      new TunableNumberGroup(IntakeConstants.ROOT_TABLE);
  // Roller
  private static final TunableNumberGroup rollerSubgroup =
      group.subgroup(RollerConstants.ROOT_TABLE);
  private static final LoggedTunableNumber rKS = rollerSubgroup.build("KS");
  private static final LoggedTunableNumber rKP = rollerSubgroup.build("KP");
  private static final LoggedTunableNumber rKV = rollerSubgroup.build("KV");
  private static final LoggedTunableNumber rollerTargetAccelerationConfig =
      rollerSubgroup.build("MaxAccelerationConstraint");

  private final HopperIO io;
  private final HopperIO.Inputs inputs = new HopperIO.Inputs(logGroup);

  private HopperState hopperState = HopperState.Idle;

  private double rollerTargetRPM;

  private ControlMode rollerControlMode = ControlMode.OPEN_LOOP;

  private Trigger stallDetected =
      new Trigger(() -> inputs.rollerVelocityRPM <= 50 && inputs.rollerAppliedVolts >= 0.5)
          .debounce(.1);

  public Hopper(HopperIO io) {
    this.io = io;

    io.setRollerVoltage(0.0);

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

      logRollerControlMode.info(rollerControlMode);

      var rhc = hashCode();
      if (rKS.hasChanged(rhc)
          || rKP.hasChanged(rhc)
          || rKV.hasChanged(rhc)
          || rollerTargetAccelerationConfig.hasChanged(rhc)) {
        setRollerConstants();
      }

      switch (hopperState) {
        case Override:
          break;

        case Idle:
          break;

        case Shooting:
          break;

        default:
          break;
      }
    }
  }

  // Setters

  private void setRollerConstants() {
    io.setRollerClosedLoopConstants(
        rKP.get(), rKV.get(), rKS.get(), rollerTargetAccelerationConfig.get());
  }

  public void setRollerPercentOut(double percent) {
    io.setRollerVoltage(percent * Constants.MAX_VOLTAGE);
    rollerControlMode = ControlMode.OPEN_LOOP;
  }

  public void setRollerVelocity(double revPerMin) {
    rollerControlMode = ControlMode.CLOSED_LOOP;
    io.setRollerVelocity(revPerMin);
    rollerTargetRPM = revPerMin;
    logRollerTargetVelocityRPM.info(rollerTargetRPM);
  }
}
