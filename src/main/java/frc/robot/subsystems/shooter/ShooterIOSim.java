// Comfort below
package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.TalonFXSim;
import frc.robot.Constants;
import frc.robot.Constants.ControlMode;

public class ShooterIOSim implements ShooterIO {
  public static final LoggerEntry.Decimal log_error = Shooter.logGroup.buildDecimal("SIM_error");
  public static final LoggerEntry.Decimal log_feedForward =
      Shooter.logGroup.buildDecimal("SIM_feedForward");
  public static final LoggerEntry.Decimal log_outputVoltage =
      Shooter.logGroup.buildDecimal("SIM_outputVoltage");
  public static final LoggerEntry.Decimal log_targetVelRadPerSec =
      Shooter.logGroup.buildDecimal("SIM_targetVelRadPerSec");
  public static final LoggerEntry.Decimal log_currentVelRadPerSec =
      Shooter.logGroup.buildDecimal("SIM_currentVelRadPerSec");
  public static final LoggerEntry.Decimal log_pitchController =
      Shooter.logGroup.buildDecimal("SIM_pitchController");
  public static final LoggerEntry.EnumValue<ControlMode> log_pivotControlMode =
      Shooter.logGroup.buildEnum("SIM_pivotControlMode");
  public static final LoggerEntry.Decimal log_targetPositionDegrees =
      Shooter.logGroup.buildDecimal("SIM_targetPositionDegrees");

  private final SingleJointedArmSim pivot =
      new SingleJointedArmSim(
          DCMotor.getFalcon500Foc(1),
          Constants.ShooterConstants.Hood.GEARING,
          SingleJointedArmSim.estimateMOI(Units.Feet.of(1.5).in(Units.Meters), 20.0),
          Constants.ShooterConstants.SHOOTER_LENGTH.in(Units.Meters),
          Constants.ShooterConstants.Hood.MIN_ANGLE_RAD.getRadians(),
          Constants.ShooterConstants.Hood.MAX_ANGLE_RAD.getRadians(),
          false,
          Constants.ShooterConstants.Hood.SIM_INITIAL_ANGLE);

  private final TalonFXSim flywheelMotorSim =
      new TalonFXSim(
          DCMotor.getKrakenX60Foc(2),
          Constants.ShooterConstants.Flywheel.GEARING,
          Constants.ShooterConstants.Flywheel.MOI);

  private final TalonFXSim kickerMotorSim =
      new TalonFXSim(
          DCMotor.getKrakenX60Foc(1),
          Constants.ShooterConstants.Kicker.GEARING,
          Constants.ShooterConstants.Kicker.MOI);

  private final ProfiledPIDController pivotFeedback =
      new ProfiledPIDController(50.0, 0, 0.0, new Constraints(5, 10.0));

  private final PIDController pivotVelController = new PIDController(60, 0, 0);
  private final double pivotTargetVelRadPerSec = 0.0;
  private ControlMode pivotControlMode = ControlMode.VELOCITY;
  private Rotation2d pivotClosedLoopTargetAngle = Constants.zeroRotation2d;
  private double pivotControlEffort = 0.0;
  private double pivotOpenLoopVolts = 0.0;
  private double kickerVolts = 0.0;

  private VoltageOut flywheelOpenLoopVolts = new VoltageOut(0.0).withEnableFOC(true);

  private double targetRPM = 0.0;

  public ShooterIOSim() {}

  @Override
  public void updateInputs(Inputs inputs) {

    pivot.update(Constants.kDefaultPeriod);
    flywheelMotorSim.update(Constants.kDefaultPeriod);
    kickerMotorSim.update(Constants.kDefaultPeriod);

    pivotFeedback.setTolerance(1.0);

    inputs.hoodPosition = new Rotation2d(pivot.getAngleRads());

    inputs.flywheelRPM[0] = targetRPM;

    inputs.kickerAppliedVolts = kickerVolts;

    double ff = 0.0;

    if (pivotControlMode.equals(ControlMode.POSITION)) {

      pivotControlEffort =
          pivotFeedback.calculate(
                  inputs.hoodPosition.getRadians(), pivotClosedLoopTargetAngle.getRadians())
              + ff;

      log_error.info(pivotFeedback.getPositionError());
    } else if (pivotControlMode.equals(ControlMode.VELOCITY)) {

      pivotControlEffort =
          pivotVelController.calculate(pivot.getVelocityRadPerSec(), pivotTargetVelRadPerSec) + ff;

    } else {

      pivotControlEffort = pivotOpenLoopVolts;
    }

    log_feedForward.info(ff);

    pivot.setInputVoltage(MathUtil.clamp(pivotControlEffort, -12.0, 12.0));
    log_outputVoltage.info(MathUtil.clamp(pivotControlEffort, -12.0, 12.0));
    log_targetVelRadPerSec.info(pivotTargetVelRadPerSec);
    log_currentVelRadPerSec.info(pivot.getVelocityRadPerSec());
    log_pitchController.info(
        pivotVelController.calculate(pivot.getVelocityRadPerSec(), pivotTargetVelRadPerSec));
    log_pivotControlMode.info(pivotControlMode);
    log_targetPositionDegrees.info(pivotClosedLoopTargetAngle.getDegrees());
  }

  @Override
  public void setHoodPosition(Rotation2d rot) {
    pivotClosedLoopTargetAngle = rot;
    pivotControlMode = ControlMode.POSITION;
  }

  @Override
  public void setHoodClosedLoopConstants(
      double kP, double kD, double kG, double maxProfiledVelocity, double maxProfiledAcceleration) {

    pivotFeedback.setP(kP);
    pivotFeedback.setD(kD);
    pivotFeedback.setConstraints(new Constraints(maxProfiledVelocity, maxProfiledAcceleration));
  }

  @Override
  public void setHoodVoltage(double volts) {
    pivotOpenLoopVolts = volts;
    pivotControlMode = ControlMode.VOLTAGE;
  }

  @Override
  public void setFlywheelVoltage(double volts) {
    flywheelMotorSim.setControl(flywheelOpenLoopVolts.withOutput(volts));
  }

  @Override
  public void setKickerVoltage(double volts) {
    VoltageOut kickerVolts = new VoltageOut(MathUtil.clamp(volts, -12.0, 12.0));
    kickerMotorSim.setControl(kickerVolts);
  }

  @Override
  public void setFlywheelRPM(double topRollerRPM, double bottomRollerRPM) {
    targetRPM = topRollerRPM;
  }
}
