package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.LinearFilter;
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
    Idle, Scoring, BumperShot, Passing, Stop, Unjamming
  }

  private final String ROOT_TABLE = "Shooter";

  private final ExecutionTiming timing = new ExecutionTiming(ROOT_TABLE);

  public static class Logs extends AutoRegisterLoggers {
    public LoggerEntry.EnumValue<Goal> state;

    public LoggerEntry.Decimal pitchDegrees;
    public LoggerEntry.Decimal targetPitchDegrees;
    public LoggerEntry.Decimal hoodPIDLatency;
    public LoggerEntry.Decimal flywheelTargetRPM;
    public LoggerEntry.Decimal flywheelRollersDifference;
    public LoggerEntry.Decimal targetDistance;
    public LoggerEntry.Decimal hoodTarget;
    public LoggerEntry.Bool    facingTarget;
    public LoggerEntry.Bool    shooterReady;
    public LoggerEntry.Bool    hoodReady;
    public LoggerEntry.Bool    flywheelReady;
    public LoggerEntry.Bool    flywheelIdleRPM;

    public LoggerEntry.Bool stallDetected;
    public LoggerEntry.Bool persistantStallDetected;

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
    public LoggerEntry.Decimal      kickerTargetRPM;
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
    public LoggedTunableNumber KP;
    public LoggedTunableNumber KI;
    public LoggedTunableNumber KD;
    public LoggedTunableNumber KS;
    public LoggedTunableNumber KV;
    public LoggedTunableNumber closedLoopMaxAccelerationConstraint;

    @TunableEntry(defaultValue = 0.0)
    public LoggedTunableNumber overrideRPM;

    @TunableEntry(defaultValue = 250)
    public LoggedTunableNumber toleranceRPM;

    public boolean hasChanged() {
      var hasChanged = false;
      hasChanged |= KP.hasChanged();
      hasChanged |= KI.hasChanged();
      hasChanged |= KD.hasChanged();
      hasChanged |= KV.hasChanged();
      hasChanged |= KS.hasChanged();
      hasChanged |= closedLoopMaxAccelerationConstraint.hasChanged();
      return hasChanged;
    }
  }

  public static class KickerTunables extends AutoRegisterTunables {
    public LoggedTunableNumber KP;
    public LoggedTunableNumber KI;
    public LoggedTunableNumber KD;
    public LoggedTunableNumber KS;
    public LoggedTunableNumber KV;
    public LoggedTunableNumber closedLoopMaxAccelerationConstraint;

    @TunableEntry(defaultValue = 0.0)
    public LoggedTunableNumber overrideRPM;

    public boolean hasChanged() {
      var hasChanged = false;
      hasChanged |= KP.hasChanged();
      hasChanged |= KI.hasChanged();
      hasChanged |= KD.hasChanged();
      hasChanged |= KV.hasChanged();
      hasChanged |= KS.hasChanged();
      hasChanged |= closedLoopMaxAccelerationConstraint.hasChanged();
      return hasChanged;
    }
  }

  public static class Tunables extends AutoRegisterTunables {
    public HoodTunables hood;

    public FlywheelTunables flywheel;

    public KickerTunables kicker;
  }

  private final Tunables tunables = TunableNumberGroup.newAutoTunable(Tunables.class, ROOT_TABLE);

  private final ShooterIO        io;
  private final ShooterIO.Inputs inputs = new ShooterIO.Inputs(logs.group);

  private final TunableInterpolationMap flywheelRpmForHubDistanceMap;
  private final TunableInterpolationMap flywheelRpmForAllianceZoneDistanceMap;
  private final TunableInterpolationMap hoodAngleForHubDistanceMap;
  private final TunableInterpolationMap hoodAngleForAllianceZoneDistanceMap;

  private boolean flywheelIdleRPMEnabled = true;

  // Creates a new flat moving average filter
  // Average will be taken over the last 20 samples
  private final LinearFilter hoodPidLatencyfilter = LinearFilter.movingAverage(20);

  private final List<PIDTargetMeasurement> hoodTargetMeasurements = new ArrayList<>();

  private Goal goal = Goal.Idle;

  private double hoodPidLatency = 0.0;

  private Rotation2d targetHoodPosition = Constants.zeroRotation2d;
  private double     flywheelTargetRPM  = 0.0;

  private Distance distanceFromTarget = Distance.ofRelativeUnits(1, Units.Meters);
  private boolean  isFacingTarget;

  private final Trigger initialStallDetected = new Trigger(
      () -> inputs.kickerVelocityRPM < ShooterConstants.Kicker.SHOOTING_KICKER_RPM * .6
          && inputs.kickerCurrentAmps >= ShooterConstants.Kicker.STALL_CURRENT_THRESHOLD)
      .debounce(ShooterConstants.Kicker.DURATION_TO_CONFIRM_STALL);

  private final Trigger stallDetected = new Trigger(() -> initialStallDetected.getAsBoolean())
      .debounce(ShooterConstants.Kicker.DURATION_TO_CONFIRM_UNSTALL, DebounceType.kFalling);

  private final Trigger persistantStallDetected = new Trigger(() -> kickerStallDetected())
      .debounce(ShooterConstants.Kicker.DURATION_TO_CONFIRM_PERSISTANT_STALL, DebounceType.kRising);

  /** Creates a new ShooterSubsystem. */
  public Shooter(ShooterIO io) {
    this.io = io;

    var hood     = tunables.hood;
    var flywheel = tunables.flywheel;
    var kicker   = tunables.kicker;

    if (Constants.RobotMode.getRobot() == RobotType.ROBOT_2026_DOUG) {
      hood.KP.initDefault(800.0);
      hood.KD.initDefault(0.0);
      hood.KG.initDefault(0.0);
      hood.closedLoopMaxVelocityConstraint.initDefault(10.0);
      hood.closedLoopMaxAccelerationConstraint.initDefault(5.0);

      flywheel.KP.initDefault(0.1);
      flywheel.KI.initDefault(0.8);
      flywheel.KD.initDefault(0.01);
      flywheel.KV.initDefault(0.0);
      flywheel.KS.initDefault(0.0);
      flywheel.closedLoopMaxAccelerationConstraint.initDefault(1000.0);

      kicker.KP.initDefault(0.1);
      kicker.KI.initDefault(4);
      kicker.KD.initDefault(0.0);
      kicker.KV.initDefault(0.0);
      kicker.KS.initDefault(0.0);
      kicker.closedLoopMaxAccelerationConstraint.initDefault(300.0);
    } else if (Constants.RobotMode.isSimBot()) {
      hood.KP.initDefault(15.0);
      hood.KD.initDefault(0.0);
      hood.KG.initDefault(0.0);
      hood.closedLoopMaxVelocityConstraint.initDefault(10.0);
      hood.closedLoopMaxAccelerationConstraint.initDefault(10.0);

      flywheel.KP.initDefault(0.1);
      flywheel.KD.initDefault(0.01);
      flywheel.KI.initDefault(1);
      flywheel.KV.initDefault(0.0);
      flywheel.KS.initDefault(0.0);
      flywheel.closedLoopMaxAccelerationConstraint.initDefault(10.0);

      kicker.KP.initDefault(0.2);
      kicker.KD.initDefault(0.0);
      kicker.KI.initDefault(0.2);
      kicker.KV.initDefault(0.0);
      kicker.KS.initDefault(0.0);
      kicker.closedLoopMaxAccelerationConstraint.initDefault(0.0);
    }

    io.setFlywheelClosedLoopConstants(flywheel);

    io.setHoodClosedLoopConstants(hood.KP.get(), hood.KD.get(), hood.KG.get(),
        hood.closedLoopMaxVelocityConstraint.get(), hood.closedLoopMaxAccelerationConstraint.get());

    io.setKickerClosedLoopConstants(kicker);

    flywheelRpmForHubDistanceMap          = ShooterConstants.Interpolation.generateFlywheelRpmForHubDistanceMap();
    flywheelRpmForAllianceZoneDistanceMap = ShooterConstants.Interpolation
        .generateFlywheelRpmForAllianceZoneDistanceMap();

    hoodAngleForHubDistanceMap          = ShooterConstants.Interpolation.generateHoodAngleForHubDistanceMap();
    hoodAngleForAllianceZoneDistanceMap = ShooterConstants.Interpolation.generateHoodAngleForAllianceZoneDistanceMap();
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
      logs.inputs.kickerTargetRPM.info(inputs.kickerTargetRPM);
      logs.inputs.tempsCelcius.info(inputs.tempsCelsius);

      logs.pitchDegrees.info(inputs.hoodPosition);
      logs.targetPitchDegrees.info(targetHoodPosition);
      logs.flywheelTargetRPM.info(flywheelTargetRPM);
      logs.flywheelRollersDifference.info(inputs.flywheelRPM[1] - inputs.flywheelRPM[0]);
      logs.state.info(goal);
      logs.targetDistance.info(distanceFromTarget.in(Units.Meters));
      logs.shooterReady.info(isShooterReady());
      logs.hoodReady.info(isHoodReady());
      logs.flywheelReady.info(isFlywheelReady());
      logs.stallDetected.info(stallDetected.getAsBoolean());
      logs.persistantStallDetected.info(persistantStallDetected.getAsBoolean());
      logs.hoodTarget.info(targetHoodPosition);
      logs.flywheelIdleRPM.info(flywheelIdleRPMEnabled);

      switch (goal) {
        case Idle:
          if (flywheelIdleRPMEnabled) {
            setFlywheelRPM(getFlywheelRPM(distanceFromTarget));
          } else {
            setFlywheelRPM(0);
          }
          setHoodPosition(ShooterConstants.Hood.IDLE_ANGLE_RAD);
          setKickerVelocity(0);
          break;

        case Passing:

        case Scoring:
          setFlywheelRPM(getFlywheelRPM(distanceFromTarget));
          setHoodPosition(getHoodAngle(distanceFromTarget));
          setKickerVelocity(getKickerRPM(isShooterReady()));
          break;

        case BumperShot:
          setFlywheelRPM(getFlywheelRPM(Distance.ofBaseUnits(1.0, Units.Meters)));
          setHoodPosition(getHoodAngle(Distance.ofBaseUnits(1.0, Units.Meters)));
          setKickerVelocity(getKickerRPM(isShooterReadyCameraless()));
          break;

        case Stop:
          setFlywheelRPM(0);
          setHoodPosition(Rotation2d.kZero);
          setKickerVelocity(0);
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

      if (flywheel.hasChanged()) {
        io.setFlywheelClosedLoopConstants(flywheel);
      }

      if (hood.KP.hasChanged() || hood.KD.hasChanged() || hood.KG.hasChanged()
          || hood.closedLoopMaxVelocityConstraint.hasChanged()
          || hood.closedLoopMaxAccelerationConstraint.hasChanged()) {

        io.setHoodClosedLoopConstants(hood.KP.get(), hood.KD.get(), hood.KG.get(),
            hood.closedLoopMaxVelocityConstraint.get(), hood.closedLoopMaxAccelerationConstraint.get());
      }

      if (kicker.hasChanged()) {
        io.setKickerClosedLoopConstants(kicker);
      }
    }
  }

  public Rotation2d getPitch() {
    return inputs.hoodPosition;
  }

  public Rotation2d getHoodAngle(Distance distance) {
    return switch (goal) {
      case Scoring -> Rotation2d.fromDegrees(hoodAngleForHubDistanceMap.get(distance.in(Units.Meters)));
      case BumperShot -> Rotation2d.fromDegrees(hoodAngleForHubDistanceMap.get(distance.in(Units.Meters)));
      case Passing -> Rotation2d.fromDegrees(hoodAngleForAllianceZoneDistanceMap.get(distance.in(Units.Meters)));
      default -> Rotation2d.kZero;
    };
  }

  public double getFlywheelRPM(Distance distance) {
    return switch (goal) {
      case Idle -> flywheelRpmForHubDistanceMap.get(distance.in(Units.Meters));
      case Scoring -> flywheelRpmForHubDistanceMap.get(distance.in(Units.Meters));
      case BumperShot -> flywheelRpmForHubDistanceMap.get(distance.in(Units.Meters));
      case Passing -> flywheelRpmForAllianceZoneDistanceMap.get(distance.in(Units.Meters));
      default -> ShooterConstants.Flywheel.IDLE_FLYWHEEL_RPM;
    };
  }

  public void setHoodPosition(Rotation2d rot) {
    io.setHoodPosition(rot);
    targetHoodPosition = rot;
  }

  public void resetHoodSensorToHomePosition() {
    io.resetHoodSensorPosition(ShooterConstants.Hood.MIN_ANGLE_RAD);
  }

  public void setFlywheelRPM(double revPerMin) {
    var overrideRPM = tunables.flywheel.overrideRPM.get();
    if (overrideRPM != 0.0) {
      revPerMin = overrideRPM;
    }
    io.setFlywheelRPM(revPerMin);
    flywheelTargetRPM = revPerMin;
  }

  public double getRPM() {
    return inputs.flywheelRPM[0];
  }

  public Distance getDistanceFromTarget() {
    if (goal == Goal.BumperShot) {
      return Distance.ofBaseUnits(1.0, Units.Meters);
    }
    return distanceFromTarget;
  }

  public double getKickerRPM(boolean shouldMoveUp) {
    if (persistantKickerStallDetected()) {
      return 0;
    }
    if (kickerStallDetected()) {
      return ShooterConstants.Kicker.UNJAM_KICKER_RPM;
    }
    if (shouldMoveUp) {
      switch (goal) {
        case Scoring:
        case Passing:
        case BumperShot:
          return ShooterConstants.Kicker.SHOOTING_KICKER_RPM;
        default:
          return 0;
      }
    }
    return 0;
  }

  public void setNeutralMode(NeutralModeValue value) {
    io.setNeutralMode(value);
  }

  public void setKickerVelocity(double revPerMin) {
    var overrideRPM = tunables.kicker.overrideRPM.get();
    if (overrideRPM != 0.0) {
      revPerMin = overrideRPM;
    }

    io.setKickerVelocity(revPerMin);
  }

  public void setDistanceFromTarget(Distance updatedDistanceFromTarget) {
    distanceFromTarget = updatedDistanceFromTarget;
  }

  public void setFacingTarget(boolean updatedIsFacingTarget) {
    isFacingTarget = updatedIsFacingTarget;
    logs.facingTarget.info(isFacingTarget);
  }

  public boolean isFlywheelReady() {
    double desiredFlywheelRPM = getFlywheelRPM(getDistanceFromTarget());
    return inputs.flywheelRPM[0] > desiredFlywheelRPM - tunables.flywheel.toleranceRPM.get();
  }

  public boolean isHoodReady() {
    double desiredHoodAngle = getHoodAngle(getDistanceFromTarget()).getDegrees();
    return Math.abs(inputs.hoodPosition.getDegrees() - desiredHoodAngle) < ShooterConstants.Hood.CORRECT_ANGLE_RANGE;
  }

  public boolean isShooterReady() {
    boolean isFacingTarget;
    isFacingTarget = true;

    return isFacingTarget && isFlywheelReady() && isHoodReady();
  }

  public boolean isShooterReadyCameraless() {
    return isFlywheelReady() && isHoodReady();
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

  public void setFlywheelIdleRPMEnabled(boolean flywheelIdle) {
    this.flywheelIdleRPMEnabled = flywheelIdle;
  }
}
