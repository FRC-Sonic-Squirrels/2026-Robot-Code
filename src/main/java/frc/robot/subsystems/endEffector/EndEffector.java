// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endEffector;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team2930.*;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.RobotMode.RobotType;
import frc.robot.RobotStates;

public class EndEffector extends SubsystemBase {
  // Execution timing
  private static final ExecutionTiming timing =
      new ExecutionTiming(EndEffectorConstants.ROOT_TABLE);

  // Logging
  private static final LoggerGroup logGroup = LoggerGroup.build(EndEffectorConstants.ROOT_TABLE);
  private static final LoggerEntry.Decimal logInputs_velocityRPM =
      logGroup.buildDecimal("VelocityRPM");
  private static final LoggerEntry.Decimal logInputs_currentAmps =
      logGroup.buildDecimal("CurrentAmps");
  private static final LoggerEntry.Decimal logInputs_tempCelsius =
      logGroup.buildDecimal("TempCelsius");
  private static final LoggerEntry.Decimal logInputs_appliedVolts =
      logGroup.buildDecimal("AppliedVolts");
  private static final LoggerEntry.Decimal logInputs_scoringSideTofDist =
      logGroup.buildDecimal("ToF/ScoringSideTofDist");
  private static final LoggerEntry.Decimal logInputs_nonScoringSideTOFDist =
      logGroup.buildDecimal("ToF/NonScoringSideTOFDist");
  private static final LoggerEntry.Bool logInputs_scoringSideTofActivated =
      logGroup.buildBoolean("ToF/ScoringSideTofActivated");
  private static final LoggerEntry.Bool logInputs_nonScoringSideTOFActivated =
      logGroup.buildBoolean("ToF/NonScoringSideTOFActivated");
  private static final LoggerEntry.Decimal logInputs_scoringSideTofSignalStrength =
      logGroup.buildDecimal("ToF/ScoringSideTofSignalStrength");
  private static final LoggerEntry.Decimal logInputs_nonScoringSideTOFSignalStrength =
      logGroup.buildDecimal("ToF/NonScoringSideTOFSignalStrength");

  private static final LoggerEntry.EnumValue<ControlMode> logControlMode =
      logGroup.buildEnum("ControlMode");

  // Tunable numbers

  private static final TunableNumberGroup group =
      new TunableNumberGroup(EndEffectorConstants.ROOT_TABLE);

  private static final LoggedTunableNumber kS = group.build("kS");
  private static final LoggedTunableNumber kP = group.build("kP");
  private static final LoggedTunableNumber kV = group.build("kV");
  private static final LoggedTunableNumber targetAccelerationConfig =
      group.build("MaxAccelerationConstraint");

  // Motion constants

  static {
    if (Constants.RobotMode.getRobot() == RobotType.ROBOT_2024_RETIRED_MAESTRO) {
      kS.initDefault(0);
      kP.initDefault(0.4);
      kV.initDefault(0.15);
      targetAccelerationConfig.initDefault(300.0);
    } else if (Constants.RobotMode.isSimBot()) {
      kS.initDefault(0);
      kP.initDefault(0.0006);
      kV.initDefault(0.0002);
      targetAccelerationConfig.initDefault(0.0);
    } else {
      kS.initDefault(0);
      kP.initDefault(0.2);
      kV.initDefault(0.115);
      targetAccelerationConfig.initDefault(400);
    }
  }

  private final EndEffectorIO io;
  private final RobotStates states;
  private final EndEffectorIO.Inputs inputs = new EndEffectorIO.Inputs(logGroup);

  private ControlMode controlMode = ControlMode.OPEN_LOOP;

  /** Creates a new EndEffector. */
  public EndEffector(EndEffectorIO io, RobotStates states) {
    this.io = io;
    this.states = states;

    setConstants();

    io.setVoltage(0.0);
  }

  public EndEffectorIOSim getSim() {
    if (io instanceof EndEffectorIOSim) {
      return (EndEffectorIOSim) io;
    }

    return null;
  }

  private int counter = 0;
  private LoggerEntry.Integer counterLog = logGroup.buildInteger("counter");

  @Override
  public void periodic() {
    try (var ignored = timing.start()) {
      counter++;
      counterLog.info(counter % 100);
      // Logging
      io.updateInputs(inputs);

      logInputs_velocityRPM.info(inputs.velocityRPM);
      logInputs_currentAmps.info(inputs.currentAmps);
      logInputs_tempCelsius.info(inputs.tempCelsius);
      logInputs_appliedVolts.info(inputs.appliedVolts);
      logInputs_scoringSideTofDist.info(inputs.scoringSideTofDistInches);
      logInputs_nonScoringSideTOFDist.info(inputs.nonScoringSideTofDistInches);
      logInputs_scoringSideTofActivated.info(inputs.scoringSideTofDetecting);
      logInputs_nonScoringSideTOFActivated.info(inputs.nonScoringSideTofDetecting);
      logInputs_scoringSideTofSignalStrength.info(inputs.scoringSideSignalStrength);
      logInputs_nonScoringSideTOFSignalStrength.info(inputs.nonScoringSideSignalStrength);

      logControlMode.info(controlMode);
    }
  }

  // Update tunable numbers

  // Setters

  private void setConstants() {
    io.setClosedLoopConstants(kP.get(), kV.get(), kS.get(), targetAccelerationConfig.get());
  }

  // Getters
  public Current getCurrentDraw() {
    return Units.Amps.of(inputs.currentAmps);
  }

  public AngularVelocity getVelocity() {
    return Units.RPM.of(inputs.velocityRPM);
  }

  public Distance scoringSideTofDistance() {
    return Units.Inches.of(inputs.scoringSideTofDistInches);
  }

  public Distance nonScoringSideTofDistance() {
    return Units.Inches.of(inputs.nonScoringSideTofDistInches);
  }

  public boolean scoringSideTofSeenGamepiece() {
    return inputs.scoringSideTofDetecting;
  }

  public boolean nonScoringSideTOFSeenGamepiece() {
    return inputs.nonScoringSideTofDetecting;
  }

  public double getMotorPosition() {
    return inputs.position;
  }
}
