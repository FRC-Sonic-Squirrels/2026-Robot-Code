// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanism;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import frc.lib.team2930.ExecutionTiming;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.robot.RobotStates;
import frc.robot.subsystems.mechanism.MechanismPositions.MechanismPosition;
import frc.robot.subsystems.mechanism.arm.Arm;
import frc.robot.subsystems.mechanism.elevator.Elevator;

/** Add your docs here. */
public class Mechanism {
  private static final String ROOT_TABLE = "Mechanism";

  // Execution timing
  private static final ExecutionTiming timing = new ExecutionTiming(ROOT_TABLE);

  // Arm and Elevator objects

  private final Elevator elevator;
  private final Arm arm;

  // Logging

  private static final LoggerGroup logGroup = LoggerGroup.build(ROOT_TABLE);

  private static final LoggerEntry.Bool log_ElevatorInPosition =
      logGroup.buildBoolean("ElevatorInPosition");
  private static final LoggerEntry.Bool log_ArmInPosition = logGroup.buildBoolean("ArmInPosition");
  private static final LoggerEntry.Text log_currentMotionState =
      logGroup.buildString("CurrentMotionState");
  private static final LoggerEntry.Text log_currentSection = logGroup.buildString("CurrentSection");
  private static final LoggerEntry.Text log_targetSection = logGroup.buildString("TargetSection");

  // States

  private boolean elevatorInPosition = false;
  private boolean armInPosition = false;
  private final RobotStates states;

  public Mechanism(Elevator elevator, Arm arm, RobotStates states) {
    this.elevator = elevator;
    this.arm = arm;
    this.states = states;
  }

  public void periodic() {
    try (var ignored = timing.start()) {
      // Enum based state machine
      switch (states.mechState) {
        case Idle:
          elevator.setPercentOut(0);
          arm.setPercentOut(0);
          break;
        case Override:
          break;
        default:
          break;
      }
      states.mechInTargetState = mechInPosition();
    }
  }

  public boolean mechInPosition() {
    return elevatorInPosition && armInPosition;
  }

  public Elevator getElevator() {
    return elevator;
  }

  public Arm getArm() {
    return arm;
  }

  private void goToPositionParallel(
      MechanismPosition position, Double elevatorAccel, Double armAccel) {

    // Mech sections/positions

    MechSection targetMechSection = getMechSection(position);
    MechanismPosition currentMechPos = new MechanismPosition(elevator.getHeight(), arm.getAngle());
    MechSection currentMechSection = getMechSection(currentMechPos);

    // Log mech sections

    log_currentSection.info(currentMechSection.name());
    log_targetSection.info(targetMechSection.name());

    if (currentMechPos.armAngle().getDegrees() > 142
        && !elevator.isAtTarget(position.elevatorHeight())
        && position.armAngle().getDegrees()
            <= 142) { // Bring arm in first if it is in danger of contacting reef
      if (armAccel.equals(Double.NaN)) {
        arm.setAngle(position.armAngle());
      } else {
        arm.setAngle(position.armAngle(), armAccel);
      }
      log_currentMotionState.info("Bring arm back");
    } else if (compatibleMechSections(
        currentMechSection,
        targetMechSection)) { // If mech sections are compatible, do direct motion
      log_currentMotionState.info("Compatible");
      goToPositionParallelSimple(position, elevatorAccel, armAccel);
    } else { // Reroute mech based on current and target mech sections
      if (currentMechSection == MechSection.S1 || currentMechSection == MechSection.S10) {
        log_currentMotionState.info("Getting out of " + currentMechSection.name());
        goToPositionParallelSimple(
            MechanismPositions.intermediateLowBackPosition(), elevatorAccel, armAccel);
      } else if (currentMechSection == MechSection.S2
          || (currentMechSection == MechSection.S3 && targetMechSection != MechSection.S1)
          || currentMechSection == MechSection.S9) {
        log_currentMotionState.info("Getting out of " + currentMechSection.name());
        goToPositionParallelSimple(
            MechanismPositions.intermediateLowPosition(), elevatorAccel, armAccel);
      } else if (currentMechSection == MechSection.S8) {
        log_currentMotionState.info("Getting out of " + currentMechSection.name());
        goToPositionParallelSimple(
            MechanismPositions.intermediateHighPosition(), elevatorAccel, armAccel);
      } else if (targetMechSection == MechSection.S2 || targetMechSection == MechSection.S3) {
        log_currentMotionState.info("Getting into " + targetMechSection.name());
        goToPositionParallelSimple(
            MechanismPositions.intermediateLowPosition(), elevatorAccel, armAccel);
      } else if (targetMechSection == MechSection.S9 || targetMechSection == MechSection.S8) {
        log_currentMotionState.info("Getting into " + targetMechSection.name());
        goToPositionParallelSimple(
            new MechanismPosition(
                position.elevatorHeight(),
                MechanismPositions.intermediateHighPosition().armAngle()),
            elevatorAccel,
            armAccel);
      } else {
        log_currentMotionState.info("Getting into S1");
        if (targetMechSection == MechSection.S1
            && (currentMechSection == MechSection.S4 || currentMechSection == MechSection.S3)) {
          goToPositionParallelSimple(
              MechanismPositions.intermediateLowBackPosition(), elevatorAccel, armAccel);
        } else {
          goToPositionParallelSimple(
              MechanismPositions.intermediateLowPosition(), elevatorAccel, armAccel);
        }
      }
    }

    // Logging

    elevatorInPosition = elevator.isAtTarget(position.elevatorHeight());
    log_ElevatorInPosition.info(elevatorInPosition);
    armInPosition = arm.isAtTargetAngle(position.armAngle());
    log_ArmInPosition.info(armInPosition);
  }

  /**
   * @param mech1
   * @param mech2
   * @return whether direct mech motion would not cause collision
   */
  private static boolean compatibleMechSections(MechSection mech1, MechSection mech2) {
    if (mech1 == mech2) return true;
    if ((mech1 == MechSection.S4 || mech1 == MechSection.S6 || mech1 == MechSection.S7)
        && (mech2 == MechSection.S4 || mech2 == MechSection.S6 || mech2 == MechSection.S7))
      return true;

    if ((mech1 == MechSection.S2 || mech1 == MechSection.S3 || mech1 == MechSection.S4)
        && (mech2 == MechSection.S2 || mech2 == MechSection.S3 || mech2 == MechSection.S4))
      return true;

    if ((mech1 == MechSection.S7 || mech1 == MechSection.S8 || mech1 == MechSection.S9)
        && (mech2 == MechSection.S7 || mech2 == MechSection.S8 || mech2 == MechSection.S9))
      return true;

    if ((mech1 == MechSection.S1 || mech1 == MechSection.S2 || mech1 == MechSection.S10)
        && (mech2 == MechSection.S1 || mech2 == MechSection.S2 || mech2 == MechSection.S10))
      return true;

    return false;
  }

  /**
   * Sends mechanism to target position with little smart routing
   *
   * @param position
   * @param elevatorAccel
   * @param armAccel
   */
  private void goToPositionParallelSimple(
      MechanismPosition position, Double elevatorAccel, Double armAccel) {

    if (!elevator.isAtTarget(position.elevatorHeight())
        && position.armAngle().getDegrees()
            > 142) { // Wait until elevator in position to bring arm all the way out
      position = new MechanismPosition(position.elevatorHeight(), Rotation2d.fromDegrees(142));
    }

    if (elevatorAccel.equals(Double.NaN)) {
      elevator.setHeight(position.elevatorHeight());
    } else {
      elevator.setHeight(position.elevatorHeight(), elevatorAccel);
    }

    if (armAccel.equals(Double.NaN)) {
      arm.setAngle(position.armAngle());
    } else {
      arm.setAngle(position.armAngle(), armAccel);
    }
  }

  private static MechSection getMechSection(MechanismPosition position) {
    if (position.armAngle().getDegrees() > 125) { // is arm back
      if (position.elevatorHeight().in(Units.Inches) < 0.1) { // is arm clear to swing under
        return MechSection.S2;
      } else if (position.elevatorHeight().in(Units.Inches)
          < 25) { // is arm not clear to swing over
        return MechSection.S1;
      } else return MechSection.S9;
    } else if (position.armAngle().getDegrees() > 75) { // is arm up
      if (position.elevatorHeight().in(Units.Inches) < 0.1) { // is arm clear to swing under
        return MechSection.S3;
      } else if (position.elevatorHeight().in(Units.Inches)
          < 25) { // is arm not clear to swing over
        if (position.armAngle().getDegrees() > 110) {
          return MechSection.S10;
        }
        return MechSection.S5;
      } else return MechSection.S8;
    } else { // arm is forward
      if (position.elevatorHeight().in(Units.Inches) < 0.1) { // is arm clear to swing under
        return MechSection.S4;
      } else if (position.elevatorHeight().in(Units.Inches)
          < 25) { // is arm not clear to swing over
        return MechSection.S6;
      } else return MechSection.S7;
    }
  }

  private enum MechSection {
    S1, // elevator above 0.1, below top tube, arm is back
    S2, // elevator below 0.1, arm is back
    S3, // elevator below top tube, arm under top tube to front side
    S4, // elevator below 0.1, arm is forward
    S5, // below top tube, elevator above 0.1, arm is toward front side
    S6, // below top tube, elevator above 0.1, arm is forward
    S7, // above top tube, arm is forward
    S8, // above top tube, arm is near straight up
    S9, // above top tube, arm is back
    S10 // below top tube, arm under top tube to back side
  }
}
