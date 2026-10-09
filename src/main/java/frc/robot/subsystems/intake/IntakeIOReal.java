package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.IntakeConstants.PivotConstants;
import frc.robot.Constants.IntakeConstants.RollerConstants;
import frc.robot.Constants.MotorConstants.KrakenConstants;
import frc.robot.subsystems.TalonFXWrapper;

public class IntakeIOReal implements IntakeIO {
  private final TalonFXWrapper rollerMotor;

  private final StatusSignal<Current>         rollerCurrent;
  private final StatusSignal<Temperature>     rollerDeviceTemp;
  private final StatusSignal<Voltage>         rollerAppliedVoltage;
  private final StatusSignal<AngularVelocity> rollerVelocity;

  private final MotionMagicVelocityVoltage rollerClosedLoopControl = new MotionMagicVelocityVoltage(0)
      .withEnableFOC(true);

  private final StatusSignal<Voltage>         pivotAppliedVoltage;
  private final StatusSignal<Angle>           pivotPosition;
  private final StatusSignal<Current>         pivotCurrent;
  private final StatusSignal<Temperature>     pivotTemp;
  private final StatusSignal<AngularVelocity> pivotVelocity;

  private final MotionMagicVoltage pivotClosedLoopControl = new MotionMagicVoltage(0.0).withEnableFOC(true);

  private final TalonFXWrapper pivotMotor;

  private final BaseStatusSignal[] refreshSet;

  private final VoltageOut pivotOpenLoopControl = new VoltageOut(0.0).withEnableFOC(true);

  public IntakeIOReal(IntakeIOConfig config) {
    pivotMotor  = config.pivotMotor();
    rollerMotor = config.rollerMotor();

    // Motor config
    rollerMotor.talonFXConfiguration.change((rollerConfig) -> {
      CurrentLimitsConfigs rollerCurrentLimitConfig = new CurrentLimitsConfigs();

      rollerCurrentLimitConfig.SupplyCurrentLimit       = RollerConstants.SUPPLY_CURRENT_LIMIT;
      rollerCurrentLimitConfig.SupplyCurrentLimitEnable = true;
      rollerCurrentLimitConfig.StatorCurrentLimit       = RollerConstants.STATOR_CURRENT_LIMIT;
      rollerCurrentLimitConfig.StatorCurrentLimitEnable = true;

      rollerConfig.CurrentLimits = rollerCurrentLimitConfig;

      rollerConfig.Feedback.SensorToMechanismRatio = RollerConstants.ROLLER_GEARING;
      rollerConfig.MotorOutput.Inverted            = config.rollerMotorInverted();
      rollerConfig.MotorOutput.NeutralMode         = NeutralModeValue.Coast;

      rollerConfig.Voltage.SupplyVoltageTimeConstant = KrakenConstants.SUPPLY_VOLTAGE_TIME;
    });

    // Status signals

    var rollerIO = rollerMotor.io;
    rollerCurrent        = rollerIO.getStatorCurrent();
    rollerDeviceTemp     = rollerIO.getDeviceTemp();
    rollerAppliedVoltage = rollerIO.getMotorVoltage();
    rollerVelocity       = rollerIO.getVelocity();

    // Update status signals

    BaseStatusSignal.setUpdateFrequencyForAll(50, rollerAppliedVoltage, rollerCurrent, rollerVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(1, rollerDeviceTemp);

    rollerIO.optimizeBusUtilization();

    pivotMotor.talonFXConfiguration.change((pivotConfig) -> {

      pivotConfig.CurrentLimits.SupplyCurrentLimit       = PivotConstants.HIGH_SUPPLY_CURRENT_LIMIT;
      pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
      pivotConfig.CurrentLimits.StatorCurrentLimit       = PivotConstants.HIGH_STATOR_CURRENT_LIMIT;
      pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;

      pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      pivotConfig.MotorOutput.Inverted    = config.pivotMotorInverted();

      pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

      pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = PivotConstants.MAX_PIVOT_ANGLE_RIGHT.getRotations();
      pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = PivotConstants.MIN_PIVOT_ANGLE
          .minus(Rotation2d.fromDegrees(2.0)).getRotations();

      pivotConfig.Feedback.SensorToMechanismRatio = PivotConstants.GEAR_RATIO;
      pivotConfig.Slot0.GravityType               = GravityTypeValue.Arm_Cosine;

      pivotConfig.Voltage.SupplyVoltageTimeConstant = KrakenConstants.SUPPLY_VOLTAGE_TIME;
    });

    pivotMotor.setNeutralMode(NeutralModeValue.Brake);

    // Status signals

    var pivotIO = pivotMotor.io;
    pivotAppliedVoltage = pivotIO.getMotorVoltage();
    pivotPosition       = pivotIO.getPosition();
    pivotCurrent        = pivotIO.getStatorCurrent();
    pivotTemp           = pivotIO.getDeviceTemp();
    pivotVelocity       = pivotIO.getVelocity();

    // Update status signals

    BaseStatusSignal.setUpdateFrequencyForAll(100, pivotAppliedVoltage, pivotPosition, pivotVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(50, pivotCurrent);
    BaseStatusSignal.setUpdateFrequencyForAll(1, pivotTemp);

    pivotIO.optimizeBusUtilization();

    refreshSet = new BaseStatusSignal[] {
        rollerCurrent,
        rollerDeviceTemp,
        rollerAppliedVoltage,
        rollerVelocity,
        pivotCurrent,
        pivotTemp,
        pivotAppliedVoltage,
        pivotVelocity,
        pivotPosition
    };
  }

  @Override
  public void updateInputs(Inputs inputs) {
    inputs.refreshAll(refreshSet);

    inputs.rollerCurrentAmps  = rollerCurrent.getValue().in(Units.Amps);
    inputs.rollerTempCelsius  = rollerDeviceTemp.getValue().in(Units.Celsius);
    inputs.rollerAppliedVolts = rollerAppliedVoltage.getValue().in(Units.Volts);
    inputs.rollerVelocityRPM  = rollerVelocity.getValue().in(Units.RPM);

    inputs.pivotPosition                 = Rotation2d.fromRotations(pivotPosition.getValue().in(Units.Rotations));
    inputs.pivotAppliedVolts             = pivotAppliedVoltage.getValue().in(Units.Volts);
    inputs.pivotCurrentAmps              = pivotCurrent.getValue().in(Units.Amps);
    inputs.pivotTempCelsius              = pivotTemp.getValue().in(Units.Celsius);
    inputs.pivotVelocityDegreesPerSecond = pivotVelocity.getValue().in(Units.DegreesPerSecond);
  }

  @Override
  public void setRollerVelocity(double revPerMin) {
    if (revPerMin != 0) {
      rollerMotor.io
          .setControl(rollerClosedLoopControl.withVelocity(Units.RPM.of(revPerMin).in(Units.RotationsPerSecond)));
    } else {
      rollerMotor.io.setControl(new VoltageOut(0.0));
    }
  }

  @Override
  public void setRollerClosedLoopConstants(Intake.RollerTunables tunables) {
    rollerMotor.slot0Configs.change((pidConfig) -> {
      pidConfig.kP = tunables.KP.get();
      pidConfig.kI = tunables.KI.get();
      pidConfig.kD = tunables.KD.get();
      pidConfig.kS = tunables.KS.get();
      pidConfig.kV = tunables.KV.get();
      pidConfig.kA = 0;
      pidConfig.kG = 0;
    });

    rollerMotor.motionMagicConfigs.change((mmConfig) -> {
      mmConfig.MotionMagicAcceleration = tunables.maxAccelerationConstraint.get();
    });
  }

  @Override
  public void setPivotClosedLoopPosition(Rotation2d angle) {
    pivotClosedLoopControl.withPosition(angle.getRotations());
    pivotMotor.io.setControl(pivotClosedLoopControl);
  }

  @Override
  public void setPivotClosedLoopConstants(Intake.PivotTunables tunables) {
    pivotMotor.slot0Configs.change((pidConfig) -> {
      pidConfig.kP = tunables.KP.get();
      pidConfig.kI = tunables.KI.get();
      pidConfig.kD = tunables.KD.get();
      pidConfig.kG = tunables.KG.get();
      pidConfig.kV = tunables.KV.get();
      pidConfig.kA = 0;
      pidConfig.kS = 0;
    });

    pivotMotor.motionMagicConfigs.change((mmConfig) -> {
      mmConfig.MotionMagicAcceleration   = tunables.maxAccelerationConstraint.get();
      mmConfig.MotionMagicCruiseVelocity = tunables.maxProfiledVelocity.get();
    });
  }

  @Override
  public void setPivotVoltage(double volts) {
    pivotMotor.io.setControl(pivotOpenLoopControl.withOutput(volts));
  }

  @Override
  public void setPivotCurrentLimits(double supplyCurrentLimit, double statorCurrentLimit) {
    pivotMotor.currentLimitsConfigs.changeIfNeeded((limitConfigs) -> {
      if (limitConfigs.SupplyCurrentLimit == supplyCurrentLimit
          && limitConfigs.StatorCurrentLimit == statorCurrentLimit) {
        return false;
      }
      limitConfigs.SupplyCurrentLimit = supplyCurrentLimit;
      limitConfigs.StatorCurrentLimit = statorCurrentLimit;
      return true;
    });
  }

  @Override
  public void resetPivotSensorPosition(Rotation2d angle) {
    pivotMotor.io.setPosition(angle.getRotations());
  }

  @Override
  public void setPivotNeutralMode(NeutralModeValue value) {
    pivotMotor.io.setNeutralMode(value);
  }
}
