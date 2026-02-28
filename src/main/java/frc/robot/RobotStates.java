package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import frc.lib.team2930.*;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.drive.RotateToFaceHelper;
import frc.robot.commands.drive.RotateToFaceHelper.RotateToFaceState;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.DrivetrainWrapper;

public class RobotStates {
  private String ROOT_TABLE = "RobotStates";

  // End Effector states
  public enum EndEffectorDesiredAction {
    Idle(false), DepotIntake(false), GroundIntake(false), ShootFuel(true), PassCoralToEndEffector(false),
    PassToIntake(false);

    public final boolean alignmentActive;

    EndEffectorDesiredAction(boolean alignmentActive) {
      this.alignmentActive = alignmentActive;
    }
  }

  // Desired mech to hold coral

  public enum RobotGoal {
    // @formatter:off
    None         (false, false),
    Unjam        (false, false),
    Score        (true,  false),
    Pass         (true,  false),
    Shoot        (true,  false),
    StowClimber  (false,  true),
    MountDismount(false,  true),
    ClimbL1      (false,  true),
    ClimbL2      (false,  true),
    ClimbL3      (false,  true),
    Lock         (false,  true);
    // @formatter:on

    public final boolean shooterActive;
    public final boolean climberActive;

    RobotGoal(boolean shooterActive, boolean climberActive) {
      this.shooterActive = shooterActive;
      this.climberActive = climberActive;
    }
  }

  public enum IntakeGoal {
    Stow, LeftIntake, RightIntake, LeftEject, RightEject, RightDown, LeftDown
  }

  // Logging

  public static class SubsystemsInTargetStatesLogs extends AutoRegisterLoggers {
    public LoggerEntry.Text goal;
    public LoggerEntry.Text intakeGoal;
    public LoggerEntry.Bool inAllianceZone;
  }

  public static class SubsystemsInUseLogs extends AutoRegisterLoggers {
    public LoggerEntry.Bool intakeInUse;
    public LoggerEntry.Bool shooterInUse;
  }

  public static class Logs extends AutoRegisterLoggers {
    public LoggerEntry.Integer counter;

    public SubsystemsInUseLogs          subsystemsInUse;
    public SubsystemsInTargetStatesLogs subsystemsInTargetStates;
  }

  private final Logs logs = LoggerGroup.root.newAutoLogger(Logs.class, ROOT_TABLE);

  // States

  private RobotGoal  goal       = RobotGoal.None;
  private IntakeGoal intakeGoal = IntakeGoal.Stow;

  // Commands

  private final Intake             leftIntake;
  private final Intake             rightIntake;
  private final DrivetrainWrapper  drivetrain;
  private final Shooter            shooter;
  private final RotateToFaceHelper rotateToFaceHelper;
  private final Hopper             hopper;
  private final Climber            climber;

  private int     periodicCounter;
  private boolean driveSlow;

  public RobotStates(Climber climber, Intake leftIntake, Intake rightIntake, Shooter shooter,
      DrivetrainWrapper drivetrain, Hopper hopper) {
    this.climber       = climber;
    this.leftIntake    = leftIntake;
    this.rightIntake   = rightIntake;
    this.drivetrain    = drivetrain;
    this.shooter       = shooter;
    this.hopper        = hopper;
    rotateToFaceHelper = new RotateToFaceHelper();
  }

  public void periodic() {
    periodicCounter = (periodicCounter + 1) % 100;
    logs.counter.info(periodicCounter);

    var currentIntakeGoal = intakeGoal;
    var currentGoal       = goal;
    var isInAllianceZone  = isInAllianceZone(drivetrain.getHubPoseEstimatorPose(true));

    if (goal.climberActive) {
      // Force intakes in.
      currentIntakeGoal = IntakeGoal.Stow;
    }

    switch (currentGoal) {
      case None:
        drivetrain.resetVelocityOverride();
        drivetrain.resetRotationOverride();
        if (intakeGoal == IntakeGoal.LeftIntake || intakeGoal == IntakeGoal.RightIntake) {
          preLoadKicker();
        } else {
          shooter.setGoal(Shooter.Goal.Idle);
          hopper.setGoal(Hopper.Goal.Idle);
        }
        climber.setGoal(Climber.Goal.Stowed);
        break;
      case Unjam:
        shooter.setGoal(Shooter.Goal.Unjamming);
        hopper.setGoal(Hopper.Goal.Unjamming);
        break;
      case Score:
        score();
        break;
      case Pass:
        pass();
        break;
      case Shoot:
        if (isInAllianceZone) {
          score();
        } else {
          pass();
        }
        break;
      case StowClimber:
        climber.setGoal(Climber.Goal.Stowed);
        break;
      case MountDismount:
        climber.setGoal(Climber.Goal.MountDismount);
        break;
      case ClimbL1:
        climber.setGoal(Climber.Goal.ClimbL1);
        break;
      case ClimbL2:
        climber.setGoal(Climber.Goal.ClimbL2);
        break;
      case ClimbL3:
        climber.setGoal(Climber.Goal.ClimbL3);
        break;
      case Lock:
        climber.setGoal(Climber.Goal.Lock);
        break;
    }

    if (goal.shooterActive) {
      shoot();
    }

    periodicIntake(currentIntakeGoal);

    // Log states

    logs.subsystemsInTargetStates.goal.info(goal.name());
    logs.subsystemsInTargetStates.intakeGoal.info(intakeGoal.name());
    logs.subsystemsInTargetStates.inAllianceZone.info(isInAllianceZone);
  }

  private void setRotateToFaceTarget(Translation2d target) {
    boolean resetPID = rotateToFaceHelper.setTarget(target);
    if (resetPID) {
      rotateToFaceHelper.resetPID(drivetrain.getHubPoseEstimatorPose(true), drivetrain.getRobotCentricVelocities());
    }
  }

  private void periodicIntake(IntakeGoal intakeGoal) {
    switch (intakeGoal) {
      case Stow:
        rightIntake.setGoal(Intake.Goal.Stow);
        leftIntake.setGoal(Intake.Goal.Stow);
        break;

      case LeftIntake:
        rightIntake.setGoal(Intake.Goal.Stow);
        if (rightIntake.isStowed()) {
          leftIntake.setGoal(Intake.Goal.Intake);
        }
        break;

      case RightIntake:
        leftIntake.setGoal(Intake.Goal.Stow);
        if (leftIntake.isStowed()) {
          rightIntake.setGoal(Intake.Goal.Intake);
        }
        break;

      case RightDown:
        leftIntake.setGoal(Intake.Goal.Stow);
        if (leftIntake.isStowed()) {
          rightIntake.setGoal(Intake.Goal.Down);
        }
        break;

      case LeftDown:
        rightIntake.setGoal(Intake.Goal.Stow);
        if (rightIntake.isStowed()) {
          leftIntake.setGoal(Intake.Goal.Down);
        }
        break;

      case LeftEject:
        leftIntake.setGoal(Intake.Goal.Eject);
        break;

      case RightEject:
        rightIntake.setGoal(Intake.Goal.Eject);
        break;
    }
  }

  private void shoot() {
    RotateToFaceState rotateState = rotateToFaceHelper.periodic(drivetrain.getHubPoseEstimatorPose(true));
    drivetrain.setRotationOverride(rotateState.omega());
    shooter.setFacingTarget(rotateState.withinTolerance());
    shooter.setDistanceFromTarget(rotateState.distance());
  }

  private void preLoadKicker() {
    shooter.setGoal(Shooter.Goal.PreLoading);
    if (!shooter.getFuelInUpperKicker()) {
      hopper.setGoal(Hopper.Goal.Shooting);
    } else {
      hopper.setGoal(Hopper.Goal.Idle);
    }
  }

  public RobotGoal getRobotGoal() {
    return goal;
  }

  public IntakeGoal getIntakeGoal() {
    return intakeGoal;
  }

  public void setRobotGoal(RobotGoal newGoal) {
    if (goal == newGoal) {
      return;
    }
    if (!newGoal.climberActive) {
      drivetrain.resetVelocityOverride();
    }
    if (!newGoal.shooterActive) {
      drivetrain.resetRotationOverride();
    }

    goal = newGoal;
  }

  public void setIntakeGoal(IntakeGoal newGoal) {
    intakeGoal = newGoal;
  }

  public TriggerOnce changeRobotGoal(RobotGoal setGoal, RobotGoal resetGoal) {
    return new TriggerOnce() {
      @Override
      public void start() {
        setRobotGoal(setGoal);
      }

      @Override
      public void end() {
        setRobotGoalIf(setGoal, resetGoal);
      }
    };
  }

  public TriggerOnce changeIntakeGoal(IntakeGoal setGoal, IntakeGoal resetGoal) {
    return new TriggerOnce() {
      @Override
      public void start() {
        setIntakeGoal(setGoal);
      }

      @Override
      public void end() {
        setIntakeGoalIf(setGoal, resetGoal);
      }
    };
  }

  public boolean setRobotGoalIf(RobotGoal expetedGoal, RobotGoal newGoal) {
    if (getRobotGoal() == expetedGoal) {
      setRobotGoal(newGoal);
      return true;
    }
    return false;
  }

  public boolean setIntakeGoalIf(IntakeGoal expetedGoal, IntakeGoal newGoal) {
    if (getIntakeGoal() == expetedGoal) {
      setIntakeGoal(newGoal);
      return true;
    }
    return false;
  }

  public boolean isInAllianceZone(Pose2d pose) {
    double positionX          = pose.getTranslation().getX();
    double fieldLength        = FieldConstants.FIELD_LENGTH.in(Units.Meters);
    double allianceZoneLength = FieldConstants.ALLIANCE_ZONE_LENGTH.in(Units.Meters);
    if (Constants.isRedAlliance()) {
      return positionX >= fieldLength - allianceZoneLength;
    } else {
      return positionX <= allianceZoneLength;
    }
  }

  public void score() {
    setRotateToFaceTarget(FieldConstants.HUB_CENTER_POSE.get());
    shooter.setGoal(Shooter.Goal.Scoring);
    hopper.setGoal(Hopper.Goal.Shooting);
  }

  public void pass() {
    Translation2d target = GeometryUtil.getClosestTranslation(FieldConstants.PASS_AIM_POINTS,
        drivetrain.getHubPoseEstimatorPose(true));
    setRotateToFaceTarget(target);
    shooter.setGoal(Shooter.Goal.Passing);
    hopper.setGoal(Hopper.Goal.Shooting);
  }
}
