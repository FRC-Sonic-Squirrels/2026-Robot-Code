// Comfort below
package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.team2930.AutoRegisterLoggers;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.TalonFXSim;
import frc.robot.Constants;
import frc.robot.Constants.ControlMode;

public class ShooterIOSim implements ShooterIO {

  public static class Logs extends AutoRegisterLoggers {
    public LoggerEntry.Decimal                error;
    public LoggerEntry.Decimal                feedForward;
    public LoggerEntry.Decimal                outputVoltage;
    public LoggerEntry.Decimal                targetVelRadPerSec;
    public LoggerEntry.Decimal                currentVelRadPerSec;
    public LoggerEntry.Decimal                pitchController;
    public LoggerEntry.EnumValue<ControlMode> pivotControlMode;
    public LoggerEntry.Decimal                targetPositionDegrees;
  }

  private static final Logs logs = Shooter.logs.group.newAutoLogger(Logs.class, "SIM");

  private final SingleJointedArmSim pivot = new SingleJointedArmSim(DCMotor.getFalcon500Foc(1),
      Constants.ShooterConstants.Hood.GEARING,
      SingleJointedArmSim.estimateMOI(Units.Feet.of(1.5).in(Units.Meters), 20.0),
      Constants.ShooterConstants.SHOOTER_LENGTH.in(Units.Meters),
      Constants.ShooterConstants.Hood.MIN_ANGLE_RAD.getRadians(),
      Constants.ShooterConstants.Hood.MAX_ANGLE_RAD.getRadians(), false,
      Constants.ShooterConstants.Hood.SIM_INITIAL_ANGLE);

  private final TalonFXSim flywheelMotorSim = new TalonFXSim(DCMotor.getKrakenX60Foc(2),
      Constants.ShooterConstants.Flywheel.GEARING, Constants.ShooterConstants.Flywheel.MOI);

  private final TalonFXSim kickerMotorSim = new TalonFXSim(DCMotor.getKrakenX60Foc(1),
      Constants.ShooterConstants.Kicker.GEARING, Constants.ShooterConstants.Kicker.MOI);

  private final ProfiledPIDController pivotFeedback = new ProfiledPIDController(50.0, 0, 0.0, new Constraints(5, 10.0));

  private final PIDController pivotVelController         = new PIDController(60, 0, 0);
  private final double        pivotTargetVelRadPerSec    = 0.0;
  private ControlMode         pivotControlMode           = ControlMode.VELOCITY;
  private Rotation2d          pivotClosedLoopTargetAngle = Constants.zeroRotation2d;
  private double              pivotControlEffort         = 0.0;
  private double              pivotOpenLoopVolts         = 0.0;
  private double              kickerVolts                = 0.0;

  private double targetRPM = 0.0;

  public ShooterIOSim() {
  }

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

      pivotControlEffort = pivotFeedback.calculate(inputs.hoodPosition.getRadians(),
          pivotClosedLoopTargetAngle.getRadians()) + ff;

      logs.error.info(pivotFeedback.getPositionError());
    } else if (pivotControlMode.equals(ControlMode.VELOCITY)) {

      pivotControlEffort = pivotVelController.calculate(pivot.getVelocityRadPerSec(), pivotTargetVelRadPerSec) + ff;

    } else {

      pivotControlEffort = pivotOpenLoopVolts;
    }

    logs.feedForward.info(ff);

    pivot.setInputVoltage(MathUtil.clamp(pivotControlEffort, -12.0, 12.0));
    logs.outputVoltage.info(MathUtil.clamp(pivotControlEffort, -12.0, 12.0));
    logs.targetVelRadPerSec.info(pivotTargetVelRadPerSec);
    logs.currentVelRadPerSec.info(pivot.getVelocityRadPerSec());
    logs.pitchController.info(pivotVelController.calculate(pivot.getVelocityRadPerSec(), pivotTargetVelRadPerSec));
    logs.pivotControlMode.info(pivotControlMode);
    logs.targetPositionDegrees.info(pivotClosedLoopTargetAngle.getDegrees());
  }

  @Override
  public void setHoodPosition(Rotation2d rot) {
    pivotClosedLoopTargetAngle = rot;
    pivotControlMode           = ControlMode.POSITION;
  }

  @Override
  public void setHoodClosedLoopConstants(double kP, double kD, double kG, double maxProfiledVelocity,
      double maxProfiledAcceleration) {

    pivotFeedback.setP(kP);
    pivotFeedback.setD(kD);
    pivotFeedback.setConstraints(new Constraints(maxProfiledVelocity, maxProfiledAcceleration));
  }

  @Override
  public void setFlywheelRPM(double revPerMin) {
    targetRPM = revPerMin;
  }
}
