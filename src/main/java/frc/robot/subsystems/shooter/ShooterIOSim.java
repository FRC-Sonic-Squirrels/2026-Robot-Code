// Comfort below
package frc.robot.subsystems.shooter;

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
import frc.lib.team2930.TalonFXSim;
import frc.robot.Constants;

public class ShooterIOSim implements ShooterIO {
  private final TalonFXSim flywheelMotorSim = new TalonFXSim(DCMotor.getKrakenX60Foc(2),
      Constants.ShooterConstants.Flywheel.GEARING, Constants.ShooterConstants.Flywheel.MOI);

  private final TalonFXSim kickerMotorSim = new TalonFXSim(DCMotor.getKrakenX60Foc(1),
      Constants.ShooterConstants.Kicker.GEARING, Constants.ShooterConstants.Kicker.MOI);

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

  public ShooterIOSim() {
  }

  @Override
  public void updateInputs(Inputs inputs) {
    flywheelMotorSim.update(Constants.kDefaultPeriod);
    kickerMotorSim.update(Constants.kDefaultPeriod);

    pivotFeedback.setTolerance(1.0);

    inputs.flywheelRPM[0]     = targetRPM;
    inputs.kickerAppliedVolts = kickerVolts;
  }

  @Override
  public void setKickerClosedLoopConstants(Shooter.KickerTunables tunables) {
    TalonFXConfiguration config       = new TalonFXConfiguration();
    Slot0Configs         slot0Configs = new Slot0Configs();

    slot0Configs.kP = tunables.KP.get();
    slot0Configs.kV = tunables.KV.get();
    slot0Configs.kS = tunables.KS.get();

    config.Slot0 = slot0Configs;
  }

  @Override
  public void setKickerVelocity(double revPerMin) {
    kickerMotorSim.setControl(kickerClosedLoopControl.withVelocity(revPerMin / 60));
  }

  @Override
  public void setFlywheelClosedLoopConstants(Shooter.FlywheelTunables tunables) {
    TalonFXConfiguration config       = new TalonFXConfiguration();
    Slot0Configs         slot0Configs = new Slot0Configs();

    slot0Configs.kP = tunables.KP.get();
    slot0Configs.kI = tunables.KI.get();
    slot0Configs.kD = tunables.KD.get();
    slot0Configs.kS = tunables.KS.get();
    slot0Configs.kV = tunables.KV.get();

    config.Slot0 = slot0Configs;
  }

  @Override
  public void setFlywheelRPM(double revPerMin) {
    flywheelMotorSim.setControl(flywheelClosedLoopControl.withVelocity(revPerMin / 60));
  }

  @Override
  public void setNeutralMode(NeutralModeValue value) {
  }
}
