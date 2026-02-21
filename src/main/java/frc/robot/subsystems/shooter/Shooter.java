package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team2930.*;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode.RobotType;
import frc.robot.Constants.ShooterConstants;

import java.util.ArrayList;
import java.util.List;

public class Shooter extends SubsystemBase {
  public enum Goal {
    Idle, Scoring, Passing, Stop, PreLoading, Unjamming
  }

  private final String ROOT_TABLE = "Shooter";

  private final ExecutionTiming timing = new ExecutionTiming(ROOT_TABLE);

  public static class Logs extends AutoRegisterLoggers {
    public LoggerEntry.EnumValue<Goal> state;

    public LoggerEntry.Decimal pitchDegrees;
    public LoggerEntry.Decimal targetPitchDegrees;
    public LoggerEntry.Decimal hoodPIDLatency;
    public LoggerEntry.Decimal flywheelRollersDifference;
    public LoggerEntry.Decimal targetDistance;
    public LoggerEntry.Bool    facingTarget;
    public LoggerEntry.Bool    shooterReady;
    public LoggerEntry.Bool    hoodReady;
    public LoggerEntry.Bool    flywheelReady;

    public InputsLogs inputs;
  }

  public static class InputsLogs extends AutoRegisterLoggers {
    public LoggerEntry.Decimal      hoodPosition;
    public LoggerEntry.Decimal      hoodVelocityRadsPerSec;
    public LoggerEntry.Decimal      hoodAppliedVolts;
    public LoggerEntry.Decimal      hoodCurrentAmps;
    public LoggerEntry.DecimalArray flywheelRPM;
    public LoggerEntry.DecimalArray flywheelAppliedVolts;
    public LoggerEntry.DecimalArray flywheelCurrentAmps;
    public LoggerEntry.Decimal      kickerAppliedVolts;
    public LoggerEntry.Decimal      kickerCurrentAmps;
    public LoggerEntry.Decimal      kickerVelocityRPM;
    public LoggerEntry.DecimalArray tempsCelcius;
  }

  public final Logs logs = LoggerGroup.root.newAutoLogger(Logs.class, ROOT_TABLE);

  public static class HoodTunables extends AutoRegisterTunables {
    public LoggedTunableNumber KP;
    public LoggedTunableNumber KD;
    public LoggedTunableNumber KG;
    public LoggedTunableNumber closedLoopMaxVelocityConstraint;
    public LoggedTunableNumber motionMagicConfigs;
    public LoggedTunableNumber closedLoopMaxAccelerationConstraint;

    @TunableEntry(defaultValue = 0.5)
    public LoggedTunableNumber toleranceDegrees;
  }

  public static class FlywheelTunables extends AutoRegisterTunables {
    public LoggedTunableNumber KS;
    public LoggedTunableNumber KP;
    public LoggedTunableNumber KV;
    public LoggedTunableNumber closedLoopMaxAccelerationConstraint;

    @TunableEntry(defaultValue = 150)
    public LoggedTunableNumber toleranceRPM;
  }

  public static class KickerTunables extends AutoRegisterTunables {
    public LoggedTunableNumber KP;
    public LoggedTunableNumber KV;
    public LoggedTunableNumber closedLoopMaxAccelerationConstraint;
  }

  public static class Tunables extends AutoRegisterTunables {
    public HoodTunables hood;

    public FlywheelTunables flywheel;

    public KickerTunables kicker;
  }

  private final Tunables tunables = TunableNumberGroup.newAutoTunable(Tunables.class, ROOT_TABLE);

  private final ShooterIO        io;
  private final ShooterIO.Inputs inputs = new ShooterIO.Inputs(logs.group);

  // Creates a new flat moving average filter
  // Average will be taken over the last 20 samples
  private final LinearFilter hoodPidLatencyfilter = LinearFilter.movingAverage(20);

  private final List<PIDTargetMeasurement> hoodTargetMeasurements = new ArrayList<>();

  private Goal goal = Goal.Idle;

  private double hoodPidLatency = 0.0;

  private Rotation2d targetHoodPosition = Constants.zeroRotation2d;
  private double     targetFlywheelRPM  = 0.0;

  private Distance distanceFromTarget = Distance.ofRelativeUnits(100, Units.Meters);
  private boolean  isFacingTarget;

  private Trigger initialStallDetected = new Trigger(() -> inputs.kickerVelocityRPM <= 50
      && inputs.kickerCurrentAmps >= ShooterConstants.Kicker.STALL_CURRENT_THRESHOLD)
      .debounce(ShooterConstants.Kicker.DURATION_TO_CONFIRM_STALL);

  private Trigger stallDetected = new Trigger(() -> initialStallDetected.getAsBoolean())
      .debounce(ShooterConstants.Kicker.DURATION_TO_CONFIRM_UNSTALL, DebounceType.kFalling);

  private Trigger persistantStallDetected = new Trigger(() -> kickerStallDetected())
      .debounce(ShooterConstants.Kicker.DURATION_TO_CONFIRM_PERSISTANT_STALL, DebounceType.kRising);

  /** Creates a new ShooterSubsystem. */
  public Shooter(ShooterIO io) {
    this.io = io;

    var hood     = tunables.hood;
    var flywheel = tunables.flywheel;
    var kicker   = tunables.kicker;

    if (Constants.RobotMode.getRobot() == RobotType.ROBOT_2026) {
      hood.KP.initDefault(800.0);
      hood.KD.initDefault(0.0);
      hood.KG.initDefault(0.0);
      hood.closedLoopMaxVelocityConstraint.initDefault(10.0);
      hood.closedLoopMaxAccelerationConstraint.initDefault(5.0);

      flywheel.KS.initDefault(0.24);
      flywheel.KP.initDefault(0.4);
      flywheel.KV.initDefault(0.072);
      flywheel.closedLoopMaxAccelerationConstraint.initDefault(1000.0);

      kicker.KP.initDefault(0.6);
      kicker.KV.initDefault(0.18);
      kicker.closedLoopMaxAccelerationConstraint.initDefault(300.0);
    } else if (Constants.RobotMode.isSimBot()) {
      hood.KP.initDefault(15.0);
      hood.KD.initDefault(0.0);
      hood.KG.initDefault(0.0);
      hood.closedLoopMaxVelocityConstraint.initDefault(10.0);
      hood.closedLoopMaxAccelerationConstraint.initDefault(10.0);

      flywheel.KS.initDefault(0.0);
      flywheel.KP.initDefault(0.5);
      flywheel.KV.initDefault(0.13);
      flywheel.closedLoopMaxAccelerationConstraint.initDefault(10.0);

      kicker.KP.initDefault(0.0);
      kicker.KV.initDefault(0.0);
      kicker.closedLoopMaxAccelerationConstraint.initDefault(0.0);
    }

    io.setFlywheelClosedLoopConstants(flywheel.KP.get(), flywheel.KV.get(), flywheel.KS.get(),
        flywheel.closedLoopMaxAccelerationConstraint.get());

    io.setHoodClosedLoopConstants(hood.KP.get(), hood.KD.get(), hood.KG.get(),
        hood.closedLoopMaxVelocityConstraint.get(), hood.closedLoopMaxAccelerationConstraint.get());

    io.setKickerClosedLoopConstants(kicker.KP.get(), kicker.KV.get(), 0.0,
        kicker.closedLoopMaxAccelerationConstraint.get());
  }

  @Override
  public void periodic() {
    try (var ignored = timing.start()) {
      io.updateInputs(inputs);
      logs.inputs.hoodPosition.info(inputs.hoodPosition);
      logs.inputs.hoodVelocityRadsPerSec.info(inputs.hoodVelocityRadsPerSec);
      logs.inputs.hoodAppliedVolts.info(inputs.hoodAppliedVolts);
      logs.inputs.hoodCurrentAmps.info(inputs.hoodCurrentAmps);
      logs.inputs.flywheelRPM.info(inputs.flywheelRPM);
      logs.inputs.flywheelAppliedVolts.info(inputs.flywheelAppliedVolts);
      logs.inputs.flywheelCurrentAmps.info(inputs.flywheelCurrentAmps);
      logs.inputs.kickerAppliedVolts.info(inputs.kickerAppliedVolts);
      logs.inputs.kickerCurrentAmps.info(inputs.kickerCurrentAmps);
      logs.inputs.kickerVelocityRPM.info(inputs.kickerVelocityRPM);
      logs.inputs.tempsCelcius.info(inputs.tempsCelcius);

      logs.pitchDegrees.info(inputs.hoodPosition);
      logs.targetPitchDegrees.info(targetHoodPosition);
      logs.flywheelRollersDifference.info(inputs.flywheelRPM[1] - inputs.flywheelRPM[0]);
      logs.state.info(goal);
      logs.targetDistance.info(distanceFromTarget.in(Units.Meters));
      logs.facingTarget.info(isFacingTarget);
      logs.shooterReady.info(isShooterReady());
      logs.hoodReady.info(isHoodReady());
      logs.flywheelReady.info(isFlywheelReady());

      switch (goal) {
        case Idle:
          setFlywheelRPM(ShooterConstants.Flywheel.IDLE_FLYWHEEL_RPM);
          setHoodPosition(ShooterConstants.Hood.IDLE_ANGLE_RAD);
          setKickerVelocity(0);
          break;

        case Passing:

        case Scoring:
          setFlywheelRPM(getFlywheelRPM(distanceFromTarget));
          setHoodPosition(getHoodAngle(distanceFromTarget));
          setKickerVelocity(getKickerRPM(isShooterReady()));
          break;

        case Stop:
          setFlywheelRPM(0);
          setHoodPosition(Rotation2d.kZero);
          setKickerVelocity(0);
          break;

        case PreLoading:
          setKickerVelocity((getKickerRPM(!getFuelInUpperKicker())));
          break;

        case Unjamming:
          setKickerVelocity(-(ShooterConstants.Kicker.SHOOTING_KICKER_RPM));
          break;
      }

      if (Constants.unusedCode) {
        var timestamp           = Timer.getFPGATimestamp();
        var hoodPositionRadians = inputs.hoodPosition.getRadians();

        hoodTargetMeasurements.add(new PIDTargetMeasurement(timestamp, targetHoodPosition,
            hoodPositionRadians <= targetHoodPosition.getRadians()));

        for (var it = hoodTargetMeasurements.iterator(); it.hasNext();) {
          var measurement = it.next();
          var targetRot   = measurement.targetRot.getRadians();

          double delta = timestamp - measurement.timestamp;
          if ((measurement.upDirection && hoodPositionRadians >= targetRot)
              || (!measurement.upDirection && hoodPositionRadians <= targetRot)) {
            hoodPidLatency = hoodPidLatencyfilter.calculate(delta);
            it.remove();
          } else if (delta >= 1.0) {
            it.remove();
          }
        }
        logs.hoodPIDLatency.info(hoodPidLatency);
      }

      var hood     = tunables.hood;
      var flywheel = tunables.flywheel;
      var kicker   = tunables.kicker;

      if (flywheel.KS.hasChanged() || flywheel.KP.hasChanged() || flywheel.KV.hasChanged()
          || flywheel.closedLoopMaxAccelerationConstraint.hasChanged()) {
        io.setFlywheelClosedLoopConstants(flywheel.KP.get(), flywheel.KV.get(), flywheel.KS.get(),
            flywheel.closedLoopMaxAccelerationConstraint.get());
      }

      if (hood.KP.hasChanged() || hood.KD.hasChanged() || hood.KG.hasChanged()
          || hood.closedLoopMaxVelocityConstraint.hasChanged()
          || hood.closedLoopMaxAccelerationConstraint.hasChanged()) {

        io.setHoodClosedLoopConstants(hood.KP.get(), hood.KD.get(), hood.KG.get(),
            hood.closedLoopMaxVelocityConstraint.get(), hood.closedLoopMaxAccelerationConstraint.get());
      }

      if (kicker.KP.hasChanged() || kicker.KV.hasChanged() || kicker.closedLoopMaxAccelerationConstraint.hasChanged()) {
        io.setKickerClosedLoopConstants(kicker.KP.get(), kicker.KV.get(), 0.0,
            kicker.closedLoopMaxAccelerationConstraint.get());
      }
    }
  }

  public Rotation2d getPitch() {
    return inputs.hoodPosition;
  }

  public Rotation2d getHoodAngle(Distance distance) {
    switch (goal) {
      case Scoring:
        return Rotation2d
            .fromDegrees(ShooterConstants.Interpolation.HOOD_ANGLE_FOR_HUB_DISTANCE.get(distance.in(Units.Meters)));
      case Passing:
        return Rotation2d.fromDegrees(
            ShooterConstants.Interpolation.HOOD_ANGLE_FOR_ALLIANCE_ZONE_DISTANCE.get(distance.in(Units.Meters)));
      default:
        return Rotation2d.kZero;
    }
  }

  public double getFlywheelRPM(Distance distance) {
    switch (goal) {
      case Scoring:
        return ShooterConstants.Interpolation.RPM_FOR_HUB_DISTANCE.get(distance.in(Units.Meters));
      case Passing:
        return ShooterConstants.Interpolation.RPM_FOR_ALLIANCE_ZONE_DISTANCE.get(distance.in(Units.Meters));
      default:
        return ShooterConstants.Flywheel.IDLE_FLYWHEEL_RPM;
    }
  }

  public void setHoodClosedLoopConstants(double kP, double kD, double kG, double maxProfiledVelocity,
      double maxProfiledAcceleration) {
    io.setHoodClosedLoopConstants(kP, kD, kG, maxProfiledVelocity, maxProfiledAcceleration);
  }

  public void setHoodPosition(Rotation2d rot) {
    io.setHoodPosition(rot);
    targetHoodPosition = rot;
  }

  public void setFlywheelRPM(double revPerMin) {
    io.setFlywheelRPM(revPerMin);
    targetFlywheelRPM = revPerMin;
  }

  public double getRPM() {
    return inputs.flywheelRPM[0];
  }

  public boolean isAtTargetRPM() {
    return isAtTargetRPM(targetFlywheelRPM);
  }

  public boolean isAtTargetRPM(double rpm) {
    return inputs.flywheelRPM[0] > rpm - tunables.flywheel.toleranceRPM.get();
  }

  public double getHoodPIDLatency() {
    return hoodPidLatency;
  }

  public boolean isHoodIsAtTarget() {
    return isHoodIsAtTarget(targetHoodPosition);
  }

  public boolean isHoodIsAtTarget(Rotation2d target) {
    return isHoodIsAtTarget(target, Rotation2d.fromDegrees(tunables.hood.toleranceDegrees.get()));
  }

  public boolean isHoodIsAtTarget(Rotation2d target, Rotation2d tolerance) {
    return Math.abs(inputs.hoodPosition.getRadians() - target.getRadians()) <= tolerance.getRadians();
  }

  public double getKickerPercentOut() {
    return inputs.kickerAppliedVolts / Constants.MAX_VOLTAGE;
  }

  public double getHoodVoltage() {
    return inputs.hoodAppliedVolts;
  }

  public double getHoodVelocity() {
    return inputs.hoodVelocityRadsPerSec;
  }

  public double getKickerRPM(boolean shouldMoveUp) {
    if (persistantKickerStallDetected()) {
      return 0;
    }
    if (kickerStallDetected()) {
      return -ShooterConstants.Kicker.SHOOTING_KICKER_RPM;
    }
    if (shouldMoveUp) {
      switch (goal) {
        case Scoring:
        case Passing:
          return ShooterConstants.Kicker.SHOOTING_KICKER_RPM;
        case PreLoading:
          return ShooterConstants.Kicker.PRELOADING_KICKER_RPM;
        default:
          return 0;
      }
    }
    return 0;
  }

  public boolean setNeutralMode(NeutralModeValue value) {
    return io.setNeutralMode(value);
  }

  public void setKickerVelocity(double revPerMin) {
    io.setKickerVelocity(revPerMin);
  }

  public void setDistanceFromTarget(Distance updatedDistanceFromTarget) {
    distanceFromTarget = updatedDistanceFromTarget;
  }

  public void setFacingTarget(boolean updatedIsFacingTarget) {
    isFacingTarget = updatedIsFacingTarget;
  }

  public boolean isFlywheelReady() {
    double desiredFlywheelRPM = getFlywheelRPM(distanceFromTarget);
    return Math.abs(inputs.flywheelRPM[0] - desiredFlywheelRPM) < ShooterConstants.Flywheel.CORRECT_RPM_RANGE;
  }

  public boolean isHoodReady() {
    double desiredHoodAngle = getHoodAngle(distanceFromTarget).getDegrees();
    return Math.abs(inputs.hoodPosition.getDegrees() - desiredHoodAngle) < ShooterConstants.Hood.CORRECT_ANGLE_RANGE;
  }

  public boolean isShooterReady() {
    return isFacingTarget && isFlywheelReady() && isHoodReady();
  }

  public boolean getFuelInUpperKicker() {
    return inputs.fuelInUpperKicker;
  }

  public boolean kickerStallDetected() {
    return stallDetected.getAsBoolean();
  }

  public boolean persistantKickerStallDetected() {
    return persistantStallDetected.getAsBoolean();
  }

  public void setGoal(Goal goal) {
    this.goal = goal;
  }
}
