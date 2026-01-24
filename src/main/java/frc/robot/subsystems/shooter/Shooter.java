package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorConstants.KrakenConstants;
import frc.robot.Robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.lib.team2930.LinearInterpolator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.PowerDistribution;

public class Shooter extends SubsystemBase {

    public enum ShooterMode {
        STOP, IDLE, SHOOTING
    }

    private ShooterMode shooterMode = ShooterMode.STOP;
    
    private double stopRPM = Constants.ShooterConstants.STOP;
    private double idleRPM = Constants.ShooterConstants.IDLE;
    private double shootingRPM = Constants.ShooterConstants.SHOOTING;

    private Robot robot;

    // Numbers from 2022 shooter subsystem
    private double desiredRpm = 0;
    private boolean atSpeed = false;
    private LinearInterpolator rpmInterpolator;
    private double currentRpm = 0;
    private double rpmDifference = 0;
    private double maxRpmDifference = 30;
    private final double rpmToTicks = 2048 / 600;

    private double teleopConfigkP = 0.2;
    private double teleopConfigkI = 0.0003;
    private double teleopCondigkD = 0.3;
    private double teleopConfigkF = 0.51;
    private double teleopConfigkIZ = 3000;
    private double maxForwardOutput = 1.0;
    private double maxReverseOutput = -0.05;
//new code
    private double rateRpmPerSecond = 10000000;
    private SlewRateLimiter rateLimiter = new SlewRateLimiter(rateRpmPerSecond);

    public Shooter(Robot robot, PowerDistribution revPhd) {
        this.robot = robot;

    }

}