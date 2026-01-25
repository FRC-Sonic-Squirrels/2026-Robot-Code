// package frc.robot.subsystems.shooter;

// import com.ctre.phoenix6.BaseStatusSignal;
// import com.ctre.phoenix6.StatusCode;
// import com.ctre.phoenix6.StatusSignal;
// import com.ctre.phoenix6.configs.MotionMagicConfigs;
// import com.ctre.phoenix6.configs.MotorOutputConfigs;
// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.Follower;
// import com.ctre.phoenix6.controls.MotionMagicVoltage;
// import com.ctre.phoenix6.controls.VoltageOut;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.MotorAlignmentValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.units.Units;
// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.units.measure.AngularVelocity;
// import edu.wpi.first.units.measure.Current;
// import edu.wpi.first.units.measure.Temperature;
// import edu.wpi.first.units.measure.Voltage;
// import frc.robot.Constants;
// import frc.robot.Constants.MotorConstants.KrakenConstants;

// public class ShooterIOReal implements ShooterIO {
//     private TalonFX flywheelLeadMotor = new TalonFX(Constants.CanIDs.SHOOTER_FLYWHEEL_LEAD_CAN_ID);
//     private TalonFX flywheelFollowMotor = new TalonFX(Constants.CanIDs.SHOOTER_FLYWHEEL_FOLLOW_CAN_ID);
//     private TalonFX pivotMotor = new TalonFX(Constants.CanIDs.SHOOTER_PIVOT_CAN_ID);
//     private TalonFX kickerMotor = new TalonFX(Constants.CanIDs.SHOOTER_KICKER_CAN_ID);

//     private final MotionMagicVoltage closedLoopControl =
//         new MotionMagicVoltage(0.0).withEnableFOC(true);
//     private final VoltageOut openLoopControl = new VoltageOut(0.0).withEnableFOC(true);

//     private StatusSignal<Angle> flywheelLeadPosition;
//     private StatusSignal<AngularVelocity> flywheelLeadVelocity;
//     private StatusSignal<Voltage> flywheelLeadVoltage;
//     private StatusSignal<Current> flywheelLeadCurrent;
//     private StatusSignal<Temperature> flywheelLeadTemp;

//     private StatusSignal<Angle> flywheelFollowPosition;
//     private StatusSignal<AngularVelocity> flywheelFollowVelocity;
//     private StatusSignal<Voltage> flywheelFollowVoltage;
//     private StatusSignal<Current> flywheelFollowCurrent;
//     private StatusSignal<Temperature> flywheelFollowTemp;

//     private final BaseStatusSignal[] refreshSet;

//     public ShooterIOReal() {

//         TalonFXConfiguration config = new TalonFXConfiguration();

//         config.CurrentLimits.SupplyCurrentLimit = Constants.ShooterConstants.SUPPLY_CURRENT_LIMIT;
//         config.CurrentLimits.SupplyCurrentLimitEnable = true;
//         config.CurrentLimits.StatorCurrentLimit = Constants.ShooterConstants.STATOR_CURRENT_LIMIT;
//         config.CurrentLimits.StatorCurrentLimitEnable = true;

//         config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
//         config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

//         config.Voltage.SupplyVoltageTimeConstant = KrakenConstants.SUPPLY_VOLTAGE_TIME;

//         flywheelLeadMotor.getConfigurator().apply(config);
//         flywheelFollowMotor.getConfigurator().apply(config);
        
//         // Status signals

//         flywheelLeadPosition = flywheelLeadMotor.getRotorPosition();
//         flywheelLeadVelocity = flywheelLeadMotor.getRotorVelocity();
//         flywheelLeadVoltage = flywheelLeadMotor.getMotorVoltage();
//         flywheelLeadCurrent = flywheelLeadMotor.getStatorCurrent();
//         flywheelLeadTemp = flywheelLeadMotor.getDeviceTemp();

//         flywheelFollowPosition = flywheelFollowMotor.getRotorPosition();
//         flywheelFollowVelocity = flywheelFollowMotor.getRotorVelocity();
//         flywheelFollowVoltage = flywheelFollowMotor.getMotorVoltage();
//         flywheelFollowCurrent = flywheelFollowMotor.getStatorCurrent();
//         flywheelFollowTemp = flywheelFollowMotor.getDeviceTemp();

//         // Update status signals

//         BaseStatusSignal.setUpdateFrequencyForAll(100, flywheelLeadPosition, flywheelLeadVelocity);
//         BaseStatusSignal.setUpdateFrequencyForAll(50, flywheelLeadVoltage, flywheelLeadCurrent);
//         BaseStatusSignal.setUpdateFrequencyForAll(1, flywheelLeadTemp);

//         BaseStatusSignal.setUpdateFrequencyForAll(100, flywheelFollowPosition, flywheelFollowVelocity);
//         BaseStatusSignal.setUpdateFrequencyForAll(50, flywheelFollowVoltage, flywheelFollowCurrent);
//         BaseStatusSignal.setUpdateFrequencyForAll(1, flywheelFollowTemp);

//         flywheelLeadMotor.optimizeBusUtilization();

//         flywheelFollowMotor.setControl(
//         new Follower(Constants.CanIDs.SHOOTER_FLYWHEEL_LEAD_CAN_ID, MotorAlignmentValue.Aligned));

//         refreshSet =
//             new BaseStatusSignal[] {flywheelLeadPosition, flywheelLeadVoltage, flywheelLeadVoltage, 
//             flywheelLeadCurrent, flywheelLeadTemp};
//   }
//     @Override
//     public void updateInputs(Inputs inputs) {
//         inputs.refreshAll(refreshSet);

//         inputs.appliedVolts = flywheelLeadVoltage.getValue().in(Units.Volts);
//         inputs.currentAmps = flywheelLeadCurrent.getValue().in(Units.Amps);
//         inputs.tempCelsius = flywheelLeadTemp.getValue().in(Units.Celsius);
//   }
//   @Override
//   public void setVoltage(double volts) {
//     openLoopControl.withOutput(volts);
//     flywheelLeadMotor.setControl(openLoopControl);
//   }
//   @Override
//   public void setClosedLoopConstants(
//       double kP, double kD, double kG, MotionMagicConfigs mmConfigs) {
//     Slot0Configs pidConfig = new Slot0Configs();

//     flywheelLeadMotor.getConfigurator().refresh(pidConfig);

//     pidConfig.kP = kP;
//     pidConfig.kD = kD;
//     pidConfig.kG = kG;

//     flywheelLeadMotor.getConfigurator().apply(pidConfig);
//     flywheelLeadMotor.getConfigurator().apply(mmConfigs);
//   }
//   @Override
//   public boolean setNeutralMode(NeutralModeValue value) {
//     var config = new MotorOutputConfigs();

//     var status1 = flywheelLeadMotor.getConfigurator().refresh(config);
//     var status2 = flywheelFollowMotor.getConfigurator().refresh(config);

//     if (status1 != StatusCode.OK || status2 != StatusCode.OK) return false;

//     config.NeutralMode = value;

//     flywheelLeadMotor.getConfigurator().apply(config);
//     flywheelFollowMotor.getConfigurator().apply(config);

//     return true;
//   }
// }
package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOReal implements ShooterIO {

  TalonFX launcher_lead = new TalonFX(Constants.CanIDs.SHOOTER_LEAD_CAN_ID);
  TalonFX launcher_follower = new TalonFX(Constants.CanIDs.SHOOTER_FOLLOW_CAN_ID);
  TalonFX pivot = new TalonFX(Constants.CanIDs.SHOOTER_PIVOT_CAN_ID);
  TalonFX kicker = new TalonFX(Constants.CanIDs.SHOOTER_KICKER_CAN_ID);

  private final StatusSignal<Double> pivotPosition;
  private final StatusSignal<Double> pivotVelocity;
  private final StatusSignal<Double> pivotVoltage;
  private final StatusSignal<Double> pivotCurrentAmps;

  private final StatusSignal<Double> launcherLeadVelocity;
  private final StatusSignal<Double> launcherLeadVoltage;
  private final StatusSignal<Double> launcherLeadCurrentAmps;
  private final StatusSignal<Double> launcherFollowVelocity;
  private final StatusSignal<Double> launcherFollowerVoltage;
  private final StatusSignal<Double> launcherFollowerCurrentAmps;

  private final StatusSignal<Double> kickerAppliedVolts;
  private final StatusSignal<Double> kickerCurrentAmps;

  private final StatusSignal<Double> launcherLeadTempCelsius;
  private final StatusSignal<Double> launcherFollowTempCelsius;
  private final StatusSignal<Double> pivotTempCelsius;
  private final StatusSignal<Double> kickerTempCelsius;
  private final StatusSignal<Double> kickerVelocity;

  private final BaseStatusSignal[] refreshSet;

  // FIX: add FOC
  private final MotionMagicVoltage pivotClosedLoopControl =
      new MotionMagicVoltage(0).withEnableFOC(true);
  private final VoltageOut pivotOpenLoop = new VoltageOut(0.0).withEnableFOC(true);

  private final MotionMagicVelocityVoltage launcherClosedLoop =
      new MotionMagicVelocityVoltage(0.0).withEnableFOC(true);
  private final VoltageOut launcherOpenLoop = new VoltageOut(0.0).withEnableFOC(true);

  private final VoltageOut kickerOpenLoop = new VoltageOut(0.0).withEnableFOC(true);

  TimeOfFlight timeOfFlight = new TimeOfFlight(Constants.CanIDs.SHOOTER_TOF_CAN_ID);

  private final MotionMagicVelocityVoltage kickerClosedLoop =
      new MotionMagicVelocityVoltage(0, 0, true, 0, 0, false, false, false);

  public ShooterIOReal() {
    // --- launcher config ---
    TalonFXConfiguration launcherConfig = new TalonFXConfiguration();

    launcherConfig.CurrentLimits.SupplyCurrentLimit = 40;
    launcherConfig.CurrentLimits.SupplyCurrentThreshold = 60.0;
    launcherConfig.CurrentLimits.SupplyTimeThreshold = 0.1;
    launcherConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    launcherConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    launcherConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    launcherConfig.Feedback.SensorToMechanismRatio = ShooterConstants.Launcher.GEARING;

    launcherConfig.Voltage.PeakReverseVoltage = 0.0;
    launcherConfig.Voltage.SupplyVoltageTimeConstant = 0.02;

    launcher_lead.getConfigurator().apply(launcherConfig);
    launcher_follower.getConfigurator().apply(launcherConfig);

    // --- pivot config ---
    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

    pivotConfig.CurrentLimits.SupplyCurrentLimit = 40;
    pivotConfig.CurrentLimits.SupplyCurrentThreshold = 60.0;
    pivotConfig.CurrentLimits.SupplyTimeThreshold = 0.1;
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    pivotConfig.Feedback.SensorToMechanismRatio = Constants.ShooterConstants.Pivot.GEARING;
    pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Constants.ShooterConstants.Pivot.MAX_ANGLE_RAD.getRotations();
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Constants.ShooterConstants.Pivot.MIN_ANGLE_RAD.getRotations();

    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    pivot.getConfigurator().apply(pivotConfig);

    // -- kicker config --
    TalonFXConfiguration kickerConfig = new TalonFXConfiguration();

    kickerConfig.CurrentLimits.SupplyCurrentLimit = 40;
    kickerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    kickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    kickerConfig.Feedback.SensorToMechanismRatio = 1.42;

    kicker.getConfigurator().apply(kickerConfig);

    pivotPosition = pivot.getPosition();
    pivotVelocity = pivot.getVelocity();
    pivotVoltage = pivot.getMotorVoltage();
    pivotCurrentAmps = pivot.getStatorCurrent();

    launcherLeadVelocity = launcher_lead.getVelocity();
    launcherFollowVelocity = launcher_follower.getVelocity();

    launcherLeadVoltage = launcher_lead.getMotorVoltage();
    launcherFollowerVoltage = launcher_follower.getMotorVoltage();
    launcherLeadCurrentAmps = launcher_lead.getStatorCurrent();
    launcherFollowerCurrentAmps = launcher_follower.getStatorCurrent();

    kickerAppliedVolts = kicker.getMotorVoltage();
    kickerCurrentAmps = kicker.getStatorCurrent();

    launcherLeadTempCelsius = launcher_lead.getDeviceTemp();
    launcherFollowTempCelsius = launcher_lead.getDeviceTemp();
    pivotTempCelsius = pivot.getDeviceTemp();
    kickerTempCelsius = kicker.getDeviceTemp();
    kickerVelocity = kicker.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100, pivotPosition, launcherLeadVelocity, launcherFollowVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        pivotVelocity,
        pivotVoltage,
        pivotCurrentAmps,
        kickerAppliedVolts,
        kickerCurrentAmps,
        kickerVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(
        10,
        launcherLeadVoltage,
        launcherLeadCurrentAmps,
        launcherFollowerVoltage,
        launcherFollowerCurrentAmps);
    BaseStatusSignal.setUpdateFrequencyForAll(
        1, launcherLeadTempCelsius, launcherFollowTempCelsius, kickerTempCelsius, pivotTempCelsius);

    launcher_lead.optimizeBusUtilization();
    launcher_follower.optimizeBusUtilization();
    pivot.optimizeBusUtilization();
    kicker.optimizeBusUtilization();

    timeOfFlight.setRangeOfInterest(0, 16, 16, 0);
    timeOfFlight.setRangingMode(RangingMode.Short, 40);

    refreshSet =
        new BaseStatusSignal[] {
          pivotPosition,
          pivotVelocity,
          pivotVoltage,
          pivotCurrentAmps,
          // --
          launcherLeadVelocity,
          launcherLeadVoltage,
          launcherLeadCurrentAmps,
          launcherFollowVelocity,
          launcherFollowerVoltage,
          launcherFollowerCurrentAmps,
          // --
          kickerAppliedVolts,
          kickerCurrentAmps,
          // --
          launcherLeadTempCelsius,
          launcherFollowTempCelsius,
          pivotTempCelsius,
          kickerTempCelsius,
          kickerVelocity
        };
  }

  @Override
  public void updateInputs(Inputs inputs) {
    BaseStatusSignal.refreshAll(refreshSet);

    inputs.pivotPosition = Rotation2d.fromRotations(pivotPosition.getValueAsDouble());
    // rotations to rads = mult by 2pi. 1 full rot = 2pi rads
    inputs.pivotVelocityRadsPerSec = pivotVelocity.getValueAsDouble() * (2 * Math.PI);
    inputs.pivotAppliedVolts = pivotVoltage.getValueAsDouble();
    inputs.pivotCurrentAmps = pivotCurrentAmps.getValueAsDouble();

    inputs.launcherRPM[0] = launcherLeadVelocity.getValueAsDouble() * 60.0;
    inputs.launcherRPM[1] = launcherFollowVelocity.getValueAsDouble() * 60.0;

    // rps to rpm = mult by 60
    inputs.launcherAppliedVolts[0] = launcherLeadVoltage.getValueAsDouble();
    inputs.launcherAppliedVolts[1] = launcherFollowerVoltage.getValueAsDouble();

    inputs.launcherCurrentAmps[0] = launcherLeadCurrentAmps.getValueAsDouble();
    inputs.launcherCurrentAmps[1] = launcherFollowerCurrentAmps.getValueAsDouble();

    inputs.kickerAppliedVolts = kickerAppliedVolts.getValueAsDouble();
    inputs.kickerCurrentAmps = kickerCurrentAmps.getValueAsDouble();
    inputs.kickerVelocityRPM = kickerVelocity.getValueAsDouble() * 60;

    inputs.tempsCelcius[0] = launcherLeadTempCelsius.getValueAsDouble();
    inputs.tempsCelcius[1] = launcherFollowTempCelsius.getValueAsDouble();
    inputs.tempsCelcius[2] = pivotTempCelsius.getValueAsDouble();
    inputs.tempsCelcius[3] = kickerTempCelsius.getValueAsDouble();

    inputs.timeOfFlightDistance = Units.Millimeters.of(timeOfFlight.getRange()).in(Units.Inches);
  }

  // PIVOT

  @Override
  public void setPivotPosition(Rotation2d rot) {
    pivotClosedLoopControl.withPosition(rot.getRotations());
    pivot.setControl(pivotClosedLoopControl);
  }

  @Override
  public void setPivotVoltage(double volts) {
    pivot.setControl(pivotOpenLoop.withOutput(volts));
  }

  // LAUNCHER
  @Override
  public void setLauncherVoltage(double volts) {
    launcher_lead.setControl(launcherOpenLoop.withOutput(volts));
    launcher_follower.setControl(launcherOpenLoop.withOutput(volts));
  }

  @Override
  public void setLauncherRPM(double topRollerRPM, double bottomRollerRPM) {
    launcher_lead.setControl(launcherClosedLoop.withVelocity(topRollerRPM / 60));
    launcher_follower.setControl(launcherClosedLoop.withVelocity(bottomRollerRPM / 60));
  }

  @Override
  public void setKickerVoltage(double volts) {
    kicker.setControl(kickerOpenLoop.withOutput(volts));
  }

  @Override
  public void resetPivotSensorPosition(Rotation2d position) {
    pivot.setPosition(position.getRotations());
  }

  @Override
  public void setPivotClosedLoopConstants(
      double kP, double kD, double kG, double maxProfiledVelocity, double maxProfiledAcceleration) {
    Slot0Configs pidConfig = new Slot0Configs();
    MotionMagicConfigs mmConfig = new MotionMagicConfigs();

    var configurator = pivot.getConfigurator();
    configurator.refresh(pidConfig);
    configurator.refresh(mmConfig);

    pidConfig.kP = kP;
    pidConfig.kD = kD;
    pidConfig.kG = kG;

    mmConfig.MotionMagicCruiseVelocity = maxProfiledVelocity;
    mmConfig.MotionMagicAcceleration = maxProfiledAcceleration;

    configurator.apply(pidConfig);
    configurator.apply(mmConfig);
  }

  @Override
  public void setLauncherClosedLoopConstants(
      double kP, double kV, double kS, double maxProfiledAcceleration) {
    Slot0Configs pidConfig = new Slot0Configs();
    MotionMagicConfigs mmConfig = new MotionMagicConfigs();

    var leadConfigurator = launcher_lead.getConfigurator();
    var followConfigurator = launcher_follower.getConfigurator();

    leadConfigurator.refresh(pidConfig);
    leadConfigurator.refresh(mmConfig);

    followConfigurator.refresh(pidConfig);
    followConfigurator.refresh(mmConfig);

    pidConfig.kP = kP;
    pidConfig.kV = kV;
    pidConfig.kS = kS;

    mmConfig.MotionMagicAcceleration = maxProfiledAcceleration;

    leadConfigurator.apply(pidConfig);
    leadConfigurator.apply(mmConfig);

    followConfigurator.apply(pidConfig);
    followConfigurator.apply(mmConfig);
  }

  @Override
  public boolean setNeutralMode(NeutralModeValue value) {
    var config = new MotorOutputConfigs();

    var status = pivot.getConfigurator().refresh(config);

    if (status != StatusCode.OK) return false;

    config.NeutralMode = value;

    pivot.getConfigurator().apply(config);
    return true;
  }

  @Override
  public void setKickerClosedLoopConstants(
      double kP, double kV, double kS, double maxProfiledAcceleration) {
    Slot0Configs pidConfig = new Slot0Configs();
    MotionMagicConfigs mmConfig = new MotionMagicConfigs();

    var config = kicker.getConfigurator();

    config.refresh(pidConfig);
    config.refresh(mmConfig);

    pidConfig.kP = kP;
    pidConfig.kV = kV;
    pidConfig.kS = kS;

    mmConfig.MotionMagicAcceleration = maxProfiledAcceleration;

    config.apply(pidConfig);
    config.apply(mmConfig);
  }

  @Override
  public void setKickerVelocity(double revPerMin) {
    kicker.setControl(kickerClosedLoop.withVelocity(revPerMin / 60.0));
  }
}