// Comfort below
// Note: is there a reason the above comment is here?
//       I found it but don't understand if it has meaning.
package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.team2930.TalonFXArmSim;
import frc.lib.team2930.TalonFXSim;
import frc.robot.Constants;

public class TurretIOSim implements TurretIO {
  private TalonFXArmSim hoodArmSim = new TalonFXArmSim(
      new SingleJointedArmSim(DCMotor.getFalcon500Foc(1), Constants.TurretConstants.Hood.GEARING,
          Constants.TurretConstants.Hood.MOI, Constants.TurretConstants.TURRET_LENGTH.in(Units.Meters),
          Constants.TurretConstants.Hood.MIN_ANGLE_RAD.getRadians(),
          Constants.TurretConstants.Hood.MAX_ANGLE_RAD.getRadians(), false,
          Constants.TurretConstants.Hood.SIM_INITIAL_ANGLE));

  private final TalonFXSim flywheelMotorSim = new TalonFXSim(DCMotor.getKrakenX60Foc(2),
      Constants.TurretConstants.Flywheel.GEARING, Constants.TurretConstants.Flywheel.MOI);

  private final TalonFXSim kickerMotorSim = new TalonFXSim(DCMotor.getKrakenX60Foc(1),
      Constants.TurretConstants.Kicker.GEARING, Constants.TurretConstants.Kicker.MOI);

  private final ProfiledPIDController pivotFeedback = new ProfiledPIDController(50.0, 0, 0.0, new Constraints(5, 10.0));

  private final PIDController hoodVelController         = new PIDController(60, 0, 0);
  private final double        hoodTargetVelRadPerSec    = 0.0;
  private Rotation2d          hoodClosedLoopTargetAngle = Constants.zeroRotation2d;
  private double              hoodControlEffort         = 0.0;
  private double              kickerVolts               = 0.0;

  private double targetRPM = 0.0;

  private MotionMagicVoltage hoodClosedLoopControl = new MotionMagicVoltage(0);

  private VelocityVoltage flywheelClosedLoopControl = new VelocityVoltage(0);

  private VelocityVoltage kickerClosedLoopControl = new VelocityVoltage(0);

  public TurretIOSim() {
  }

  @Override
  public void updateInputs(Inputs inputs) {
    hoodArmSim.update(Constants.kDefaultPeriod);
    flywheelMotorSim.update(Constants.kDefaultPeriod);
    kickerMotorSim.update(Constants.kDefaultPeriod);

    pivotFeedback.setTolerance(1.0);

    inputs.hoodPosition       = new Rotation2d(hoodArmSim.getPosition());
    inputs.flywheelRPM[0]     = targetRPM;
    inputs.kickerAppliedVolts = kickerVolts;
  }

  @Override
  public void setKickerClosedLoopConstants(Turret.KickerTunables tunables) {
    TalonFXConfiguration config       = new TalonFXConfiguration();
    Slot0Configs         slot0Configs = new Slot0Configs();

    slot0Configs.kP = tunables.KP.get();
    slot0Configs.kV = tunables.KV.get();
    slot0Configs.kS = tunables.KS.get();

    config.Slot0 = slot0Configs;

    hoodArmSim.setConfig(config);
  }

  @Override
  public void setKickerVelocity(double revPerMin) {
    kickerMotorSim.setControl(kickerClosedLoopControl.withVelocity(revPerMin / 60));
  }

  @Override
  public void setHoodPosition(Rotation2d angle) {
    hoodArmSim.setControl(hoodClosedLoopControl.withPosition(angle.getRotations()));
  }

  @Override
  public void setHoodClosedLoopConstants(double kP, double kD, double kG, double maxProfiledVelocity,
      double maxProfiledAcceleration) {
    TalonFXConfiguration config       = new TalonFXConfiguration();
    Slot0Configs         slot0Configs = new Slot0Configs();

    slot0Configs.kP = kP;
    slot0Configs.kD = kD;
    slot0Configs.kG = kG;

    config.Slot0 = slot0Configs;

    hoodArmSim.setConfig(config);

  }

  @Override
  public void resetHoodSensorPosition(Rotation2d angle) {
    hoodArmSim.setState(angle.getRadians(), 0.0);
  }

  @Override
  public void setFlywheelClosedLoopConstants(Turret.FlywheelTunables tunables) {
    TalonFXConfiguration config       = new TalonFXConfiguration();
    Slot0Configs         slot0Configs = new Slot0Configs();

    slot0Configs.kP = tunables.KP.get();
    slot0Configs.kI = tunables.KI.get();
    slot0Configs.kD = tunables.KD.get();
    slot0Configs.kS = tunables.KS.get();
    slot0Configs.kV = tunables.KV.get();

    config.Slot0 = slot0Configs;

    hoodArmSim.setConfig(config);
  }

  @Override
  public void setFlywheelRPM(double revPerMin) {
    flywheelMotorSim.setControl(flywheelClosedLoopControl.withVelocity(revPerMin / 60));
  }

  @Override
  public void setNeutralMode(NeutralModeValue value) {
  }
}
