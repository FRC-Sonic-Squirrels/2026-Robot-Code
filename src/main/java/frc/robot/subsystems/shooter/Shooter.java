package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.*;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode.RobotType;
import frc.robot.Constants.ShooterConstants;

import java.util.ArrayList;
import java.util.List;

public class Shooter extends SubsystemBase {

  public enum Goal {
    Idle, Shooting, Passing, Stop
  }

  private static final String ROOT_TABLE = "Shooter";

  private static final ExecutionTiming timing = new ExecutionTiming(ROOT_TABLE);

  static final LoggerGroup         logGroup  = LoggerGroup.build(ROOT_TABLE);
  private static final LoggerGroup logInputs = logGroup.subgroup("Inputs");

  private static final LoggerEntry.Decimal      logInputs_hoodPosition           = logInputs
      .buildDecimal("HoodPosition");
  private static final LoggerEntry.Decimal      logInputs_hoodVelocityRadsPerSec = logInputs
      .buildDecimal("HoodVelocityRadsPerSec");
  private static final LoggerEntry.Decimal      logInputs_hoodAppliedVolts       = logInputs
      .buildDecimal("HoodAppliedVolts");
  private static final LoggerEntry.Decimal      logInputs_hoodCurrentAmps        = logInputs
      .buildDecimal("HoodCurrentAmps");
  private static final LoggerEntry.DecimalArray logInputs_flywheelRPM            = logInputs
      .buildDecimalArray("FlywheelRPM");
  private static final LoggerEntry.DecimalArray logInputs_flywheelAppliedVolts   = logInputs
      .buildDecimalArray("FlywheelAppliedVolts");
  private static final LoggerEntry.DecimalArray logInputs_flywheelCurrentAmps    = logInputs
      .buildDecimalArray("FlywheelCurrentAmps");
  private static final LoggerEntry.Decimal      logInputs_kickerAppliedVolts     = logInputs
      .buildDecimal("KickerAppliedVolts");
  private static final LoggerEntry.Decimal      logInputs_kickerCurrentAmps      = logInputs
      .buildDecimal("KickerCurrentAmps");
  private static final LoggerEntry.Decimal      logInputs_kickerVelocityRPM      = logInputs
      .buildDecimal("KickerVelocityRPM");
  private static final LoggerEntry.DecimalArray logInputs_tempsCelcius           = logInputs
      .buildDecimalArray("TempsCelcius");

  private static final LoggerEntry.EnumValue<Goal> logState = logGroup.buildEnum("State");

  private static final LoggerEntry.Decimal logPitchDegrees              = logGroup.buildDecimal("PitchDegrees");
  private static final LoggerEntry.Decimal logTargetPitchDegrees        = logGroup.buildDecimal("TargetPitchDegrees");
  private static final LoggerEntry.Decimal logHoodPIDLatency            = logGroup.buildDecimal("HoodPIDLatency");
  private static final LoggerEntry.Decimal logFlywheelRollersDifference = logGroup
      .buildDecimal("FlywheelRollersDifference");
  private static final LoggerEntry.Decimal logTargetDistance            = logGroup.buildDecimal("DistanceFromTarget");
  private static final LoggerEntry.Bool    logFacingTarget              = logGroup.buildBoolean("IsFacingTarget");
  private static final LoggerEntry.Bool    logShooterReady              = logGroup.buildBoolean("IsShooterReady");

  public static final TunableNumberGroup group = new TunableNumberGroup(ROOT_TABLE);

  private static final TunableNumberGroup  groupHood                               = group.subgroup("Hood");
  private static final LoggedTunableNumber hoodkP                                  = groupHood.build("kP");
  private static final LoggedTunableNumber hoodkD                                  = groupHood.build("kD");
  private static final LoggedTunableNumber hoodkG                                  = groupHood.build("kG");
  private static final LoggedTunableNumber hoodClosedLoopMaxVelocityConstraint     = groupHood
      .build("ClosedLoopMaxVelocityConstraint");
  private static final LoggedTunableNumber hoodClosedLoopMaxAccelerationConstraint = groupHood
      .build("ClosedLoopMaxAccelerationConstraint");

  private static final TunableNumberGroup  groupFlywheel                               = group.subgroup("Flywheel");
  private static final LoggedTunableNumber flywheelkS                                  = groupFlywheel.build("kS");
  private static final LoggedTunableNumber flywheelkP                                  = groupFlywheel.build("kP");
  private static final LoggedTunableNumber flywheelkV                                  = groupFlywheel.build("kV");
  private static final LoggedTunableNumber flywheelClosedLoopMaxAccelerationConstraint = groupFlywheel
      .build("ClosedLoopMaxAccelerationConstraint");

  private static final TunableNumberGroup  groupKicker                               = group.subgroup("Kicker");
  private static final LoggedTunableNumber kickerkP                                  = groupKicker.build("kP");
  private static final LoggedTunableNumber kickerkV                                  = groupKicker.build("kV");
  private static final LoggedTunableNumber kickerClosedLoopMaxAccelerationConstraint = groupKicker
      .build("ClosedLoopMaxAccelerationConstraint");

  private static final LoggedTunableNumber hoodToleranceDegrees = group.build("hoodToleranceDegrees", 0.5);
  private static final LoggedTunableNumber flywheelToleranceRPM = group.build("flywheelToleranceRPM", 150);

  static {
    if (Constants.RobotMode.getRobot() == RobotType.ROBOT_2026) {
      hoodkP.initDefault(800.0);
      hoodkD.initDefault(0.0);
      hoodkG.initDefault(0.0);
      hoodClosedLoopMaxVelocityConstraint.initDefault(10.0);
      hoodClosedLoopMaxAccelerationConstraint.initDefault(5.0);

      flywheelkS.initDefault(0.24);
      flywheelkP.initDefault(0.4);
      flywheelkV.initDefault(0.072);
      flywheelClosedLoopMaxAccelerationConstraint.initDefault(1000.0);

      kickerkP.initDefault(0.6);
      kickerkV.initDefault(0.18);
      kickerClosedLoopMaxAccelerationConstraint.initDefault(300.0);
    } else if (Constants.RobotMode.isSimBot()) {
      hoodkP.initDefault(15.0);
      hoodkD.initDefault(0.0);
      hoodkG.initDefault(0.0);
      hoodClosedLoopMaxVelocityConstraint.initDefault(10.0);
      hoodClosedLoopMaxAccelerationConstraint.initDefault(10.0);

      flywheelkS.initDefault(0.0);
      flywheelkP.initDefault(0.5);
      flywheelkV.initDefault(0.13);
      flywheelClosedLoopMaxAccelerationConstraint.initDefault(10.0);

      kickerkP.initDefault(0.0);
      kickerkV.initDefault(0.0);
      kickerClosedLoopMaxAccelerationConstraint.initDefault(0.0);
    }
  }

  private final ShooterIO        io;
  private final ShooterIO.Inputs inputs = new ShooterIO.Inputs(logGroup);

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

  /** Creates a new ShooterSubsystem. */
  public Shooter(ShooterIO io) {
    this.io = io;

    io.setFlywheelClosedLoopConstants(flywheelkP.get(), flywheelkV.get(), flywheelkS.get(),
        flywheelClosedLoopMaxAccelerationConstraint.get());

    io.setHoodClosedLoopConstants(hoodkP.get(), hoodkD.get(), hoodkG.get(), hoodClosedLoopMaxVelocityConstraint.get(),
        hoodClosedLoopMaxAccelerationConstraint.get());

    io.setKickerClosedLoopConstants(kickerkP.get(), kickerkV.get(), 0.0,
        kickerClosedLoopMaxAccelerationConstraint.get());
  }

  @Override
  public void periodic() {
    try (var ignored = timing.start()) {
      io.updateInputs(inputs);
      logInputs_hoodPosition.info(inputs.hoodPosition);
      logInputs_hoodVelocityRadsPerSec.info(inputs.hoodVelocityRadsPerSec);
      logInputs_hoodAppliedVolts.info(inputs.hoodAppliedVolts);
      logInputs_hoodCurrentAmps.info(inputs.hoodCurrentAmps);
      logInputs_flywheelRPM.info(inputs.flywheelRPM);
      logInputs_flywheelAppliedVolts.info(inputs.flywheelAppliedVolts);
      logInputs_flywheelCurrentAmps.info(inputs.flywheelCurrentAmps);
      logInputs_kickerAppliedVolts.info(inputs.kickerAppliedVolts);
      logInputs_kickerCurrentAmps.info(inputs.kickerCurrentAmps);
      logInputs_kickerVelocityRPM.info(inputs.kickerVelocityRPM);
      logInputs_tempsCelcius.info(inputs.tempsCelcius);

      logPitchDegrees.info(inputs.hoodPosition.getDegrees());
      logTargetPitchDegrees.info(targetHoodPosition.getDegrees());
      logFlywheelRollersDifference.info(inputs.flywheelRPM[1] - inputs.flywheelRPM[0]);
      logState.info(goal);
      logTargetDistance.info(distanceFromTarget.in(Units.Meters));
      logFacingTarget.info(isFacingTarget);
      logShooterReady.info(isShooterReady());

      switch (goal) {
        case Idle:
          setFlywheelRPM(ShooterConstants.Flywheel.IDLE_FLYWHEEL_RPM);
          setHoodPosition(ShooterConstants.Hood.IDLE_ANGLE_RAD);
          setKickerVelocity(0);
          break;

        case Shooting:
          setFlywheelRPM(getFlywheelRPM(distanceFromTarget));
          setHoodPosition(getHoodAngle(distanceFromTarget));
          if (isShooterReady()) {
            setKickerVelocity(ShooterConstants.Kicker.SHOOTING_KICKER_RPM);
          } else {
            setKickerVelocity(0);
          }
          break;

        case Passing:
          break;

        case Stop:
          setFlywheelRPM(0);
          setHoodPosition(Rotation2d.kZero);
          setKickerVelocity(0);
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
        logHoodPIDLatency.info(hoodPidLatency);
      }

      var hc = hashCode();
      if (flywheelkS.hasChanged(hc) || flywheelkP.hasChanged(hc) || flywheelkV.hasChanged(hc)
          || flywheelClosedLoopMaxAccelerationConstraint.hasChanged(hc)) {
        io.setFlywheelClosedLoopConstants(flywheelkP.get(), flywheelkV.get(), flywheelkS.get(),
            flywheelClosedLoopMaxAccelerationConstraint.get());
      }

      if (hoodkP.hasChanged(hc) || hoodkD.hasChanged(hc) || hoodkG.hasChanged(hc)
          || hoodClosedLoopMaxVelocityConstraint.hasChanged(hc)
          || hoodClosedLoopMaxAccelerationConstraint.hasChanged(hc)) {

        io.setHoodClosedLoopConstants(hoodkP.get(), hoodkD.get(), hoodkG.get(),
            hoodClosedLoopMaxVelocityConstraint.get(), hoodClosedLoopMaxAccelerationConstraint.get());
      }

      if (kickerkP.hasChanged(hc) || kickerkV.hasChanged(hc)
          || kickerClosedLoopMaxAccelerationConstraint.hasChanged(hc)) {
        io.setKickerClosedLoopConstants(kickerkP.get(), kickerkV.get(), 0.0,
            kickerClosedLoopMaxAccelerationConstraint.get());
      }
    }
  }

  public Rotation2d getPitch() {
    return inputs.hoodPosition;
  }

  public static Rotation2d getHoodAngle(Distance distance) {
    if (distance.lt(ShooterConstants.Hood.MIN_DISTANCE)) {
      return Rotation2d.fromDegrees(0.0);
    }
    return Rotation2d
        .fromDegrees(ShooterConstants.Interpolation.HOOD_ANGLE_FOR_HUB_DISTANCE.get(distance.in(Units.Meters)));
  }

  public static double getFlywheelRPM(Distance distance) {
    return ShooterConstants.Interpolation.RPM_FOR_HUB_DISTANCE.get(distance.in(Units.Meters));
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
    return inputs.flywheelRPM[0] > rpm - flywheelToleranceRPM.get();
  }

  public double getHoodPIDLatency() {
    return hoodPidLatency;
  }

  public boolean isHoodIsAtTarget() {
    return isHoodIsAtTarget(targetHoodPosition);
  }

  public boolean isHoodIsAtTarget(Rotation2d target) {
    return isHoodIsAtTarget(target, Rotation2d.fromDegrees(hoodToleranceDegrees.get()));
  }

  public boolean isHoodIsAtTarget(Rotation2d target, Rotation2d tolerance) {
    return Math.abs(inputs.hoodPosition.getRadians() - target.getRadians()) <= tolerance.getRadians();
  }

  public void setKickerPercentOut(double percent) {
    io.setKickerVoltage(percent * Constants.MAX_VOLTAGE);
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

  public void setGoal(Goal goal) {
    this.goal = goal;
  }
}
