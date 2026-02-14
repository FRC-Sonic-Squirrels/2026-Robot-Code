// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.team2930.AllianceAwareTranslation2d;
import frc.robot.Constants.RobotMode.RobotType;
import frc.robot.configs.*;

import java.util.function.Supplier;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double kDefaultPeriod = 0.02;
  public static final Translation2d zeroTranslation2d = new Translation2d();
  public static final Rotation2d zeroRotation2d = new Rotation2d();
  public static final Pose2d zeroPose2d = new Pose2d();
  public static final Pose3d zeroPose3d = new Pose3d();

  public static boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
  }

  public static class RobotMode {
    private static final RobotType ROBOT = RobotType.ROBOT_2026;

    public static boolean isSimBot() {
      switch (getRobot()) {
        case ROBOT_SIMBOT:
        case ROBOT_SIMBOT_REAL_CAMERAS:
          return true;

        default:
          return false;
      }
    }

    public static Mode getMode() {
      if (isSimBot()) {
        return Mode.SIM;
      }

      return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
    }

    public static RobotType getRobot() {
      if (!RobotBase.isReal()) {
        return RobotType.ROBOT_SIMBOT;
      }
      return ROBOT;
    }

    public enum RobotType {
      // use supplier because if we just create the object, the fields in the
      // config classes are also created. Meaning tunableNumber values are stuck to
      // the first
      // object that is created. In this case ExampleRobotConfig. Suppliers solve this
      // by only creating the specific config object corresponding to the robot type
      ROBOT_SIMBOT(SimulatorRobotConfig::new),
      ROBOT_SIMBOT_REAL_CAMERAS(SimulatorRobotConfig::new),
      ROBOT_2023_RETIRED_ROBER(RobotConfig2023::new),
      ROBOT_2024_RETIRED_MAESTRO(RobotConfig2024::new),
      ROBOT_2025_RETIRED_HOLO(RobotConfig2025::new),
      ROBOT_2026(RobotConfig2026::new); // Someone please name it...

      public final Supplier<RobotConfig> config;

      RobotType(Supplier<RobotConfig> config) {
        this.config = config;
      }
    }

    public enum Mode {
      REAL,
      REPLAY,
      SIM
    }
  }

  public static double MAX_VOLTAGE = 12.0;

  public static class RobotDimensions {
    public static Distance BUMPER_THICKNESS = Units.Inches.of(4.1);

    /**
     * x = left to right
     *
     * <p>y = front to back
     */
    public static Translation2d ROBOT_DIMENSIONS_WITHOUT_BUMPERS() {
      if (RobotMode.getRobot() == RobotType.ROBOT_2024_RETIRED_MAESTRO) {
        return new Translation2d(Units.Inches.of(27.0), Units.Inches.of(32.5));
      } else {
        return new Translation2d(Units.Inches.of(28.5), Units.Inches.of(30.5));
      }
    }

    /**
     * x = left to right
     *
     * <p>y = front to back
     */
    public static Translation2d ROBOT_DIMENSIONS_WITH_BUMPERS =
        ROBOT_DIMENSIONS_WITHOUT_BUMPERS()
            .plus(new Translation2d(BUMPER_THICKNESS.times(2.0), BUMPER_THICKNESS.times(2.0)));
  }

  public static class FieldConstants {
    // Official Field dimensions from field dimensions manual
    public static Distance FIELD_LENGTH = Units.Inches.of(650.12);
    public static Distance FIELD_WIDTH = Units.Inches.of(316.64);

    public static Distance HUB_DIST_FROM_WALL = Units.Inches.of(158.06);

    public static Distance HUB_WIDTH = Units.Inches.of(41.73);
    public static Distance HUB_LENGTH = Units.Inches.of(47.00);

    public static Distance HUB_DIAGONAL_WIDTH = HUB_WIDTH.times(2.0 / Math.sqrt(3));

    public static AllianceAwareTranslation2d HUB_CENTER_POSE =
        new AllianceAwareTranslation2d(new Translation2d(HUB_DIST_FROM_WALL.plus(HUB_WIDTH.div(2.0)), FIELD_WIDTH.div(2.0)));

    public static AllianceAwareTranslation2d RIGHT_PASS_AIM_POINT =
        new AllianceAwareTranslation2d(new Translation2d(FIELD_LENGTH.div(4.0), FIELD_WIDTH.times(1.0 / 4.0)));
    public static AllianceAwareTranslation2d LEFT_PASS_AIM_POINT =
        new AllianceAwareTranslation2d(new Translation2d(FIELD_LENGTH.div(4.0), FIELD_WIDTH.times(3.0 / 4.0)));

    public static Distance REEF_BRANCH_OFFSET = Units.Inches.of(6.5);

    public static final Distance CORAL_STATION_WIDTH = Units.Inches.of(76);

    public static class Gamepieces {
      public static final Distance
          CORAL_OUTER_DIAMETER = // NOTICE: changed to fake value for improved alignment
          Units.Inches.of(8.0);
      public static final Distance GAMEPIECE_TOLERANCE = Units.Inches.of(20.0);
      public static final double GAMEPIECE_PERSISTENCE = 0.5;
    }
  }

  public static class MotorConstants {
    public static class KrakenConstants {
      public static final double MAX_RPM = 6000.0;
      public static final double NOMINAL_VOLTAGE_VOLTS = 12.0;
      public static final double STALL_TORQUE_NEWTON_METERS = 7.09;
      public static final double STALL_CURRENT_AMPS = 40.0;
      public static final double FREE_CURRENT_AMPS = 30.0;
      public static final double FREE_SPEED_RPM = 6000.0;
      public static final double SUPPLY_VOLTAGE_TIME = 0.02;
    }
  }

  public static class IntakeConstants {
    public static final String ROOT_TABLE = "Intake";

    public static class RollerConstants {

      public static final double ROLLER_GEARING = 10.4167;
      public static final double MOI = 0.05;

      public static final double SUPPLY_CURRENT_LIMIT = 20.0;
      public static final double STATOR_CURRENT_LIMIT = 100.0;

      public static final String ROOT_TABLE = "Rollers";
    }

    public static class PivotConstants {
      public static final double SUPPLY_CURRENT_LIMIT = 20.0;
      public static final double STATOR_CURRENT_LIMIT = 100.0;

      public static final double GEAR_RATIO = 57.9586;

      public static final double MOI = .00001;

      public static final Rotation2d MAX_PIVOT_ANGLE = Rotation2d.fromDegrees(129);
      public static final Rotation2d MIN_PIVOT_ANGLE = Rotation2d.fromDegrees(0);
      public static final Rotation2d HOME_POSITION = Rotation2d.fromDegrees(124.19);

      public static final Rotation2d PASSOFF_PIVOT_ANGLE = Rotation2d.fromDegrees(92.5);

      public static final Rotation2d INITIAL_PIVOT_ANGLE = Rotation2d.fromDegrees(120);
      public static final Rotation2d PIVOT_STOWED_ANGLE = Rotation2d.fromDegrees(105);

      public static final Rotation2d PIVOT_SAFE_ANGLE = Rotation2d.fromDegrees(90);

      public static final Rotation2d INTERNAL_ANGLE = Rotation2d.fromDegrees(0);
      public static final Rotation2d EXTERNAL_ANGLE = Rotation2d.fromDegrees(180);
      public static final Rotation2d ALGAE_SCORE_ANGLE = Rotation2d.fromDegrees(45);

      public static final Distance PIVOT_LENGTH = Units.Inches.of(6.215);

      public static final String ROOT_TABLE = "Pivot";
    }
  }

  public static class EndEffectorConstants {
    public static final double INTAKING_PERCENT_OUT = 1.0;

    public static final double GEARING = 14.0741;
    public static final double MOI = 0.05;

    public static final double SUPPLY_CURRENT_LIMIT = 40.0;
    public static final double STATOR_CURRENT_LIMIT = 80.0;
    public static final String ROOT_TABLE = "EndEffector";
  }

  public static class ElevatorConstants {
    public static final double GEAR_RATIO =
        RobotMode.ROBOT == RobotType.ROBOT_2024_RETIRED_MAESTRO ? 23.05 : 6.4935;
    public static final Distance PULLEY_DIAMETER = Units.Inches.of(2.35);
    public static final double CARRIAGE_MASS = 2.0; // arbitrary

    public static final double INCHES_TO_MOTOR_ROT =
        Constants.ElevatorConstants.GEAR_RATIO
            / (Math.PI * Constants.ElevatorConstants.PULLEY_DIAMETER.in(Units.Inches));

    public static final Distance MAX_HEIGHT =
        RobotMode.ROBOT == RobotType.ROBOT_2024_RETIRED_MAESTRO
            ? Units.Inches.of(26.2)
            : Units.Inches.of(60);

    public static final Distance SAFE_HEIGHT = Units.Inches.of(15.491);
    public static final double SUPPLY_CURRENT_LIMIT = 40.0;
    public static final double STATOR_CURRENT_LIMIT = 100.0;

    public static final Distance HOME_POSITION = Units.Inches.of(26.96);

    public static final Distance ELEVATOR_SAFETY_MARGIN = Units.Inches.of(1.5);
    public static final String ROOT_TABLE = "Elevator";

    public static final InterpolatingDoubleTreeMap SPEED_SCALAR_MAP =
        new InterpolatingDoubleTreeMap();

    static {
      SPEED_SCALAR_MAP.put(HOME_POSITION.in(Units.Inch), 1.0);
      SPEED_SCALAR_MAP.put(MAX_HEIGHT.in(Units.Inch), 0.8);
    }
  }

  public static class HopperConstants {
    public static class RollerConstants {

      public static final double ROLLER_GEARING = 10.4167;
      public static final double MOI = 0.05;

      public static final double SUPPLY_CURRENT_LIMIT = 20.0;
      public static final double STATOR_CURRENT_LIMIT = 100.0;

      public static final String ROLLER = "Rollers";
      public static final int SHOOTING_ROLLER_VELOCITY = 240;
    }
  }

  public static class LEDConstants {
    public static final int PWM_PORT = 9;
    public static final int MAX_LED_LENGTH = 60;
    public static final int LED_BUFFER = 36;
  }

  public static class CanIDs {
    // READ ME: CAN ID's THAT ARE NOT VALID TO USE
    // 1, 11, 21, 31
    // 2, 12, 22, 32
    // 3, 13, 23, 33
    // 4, 14, 24, 34
    // all these CAN ID's are reserved for the Drivetrain

    public static final int ARM_CAN_ID = 17;

    public static final int GYRO_2_CAN_ID = 41;

    public static final int CLIMBER_WINCH_CAN_ID = 40;

    public static final int INTAKE_LEFT_PIVOT_CAN_ID = 19;
    public static final int INTAKE_RIGHT_ROLLER_CAN_ID = 20;
    public static final int INTAKE_RIGHT_PIVOT_CAN_ID = 43;
    public static final int INTAKE_LEFT_ROLLER_CAN_ID = 21;

    public static final int SHOOTER_LEAD_CAN_ID = 35;
    public static final int SHOOTER_FOLLOW_CAN_ID = 36;
    public static final int SHOOTER_HOOD_CAN_ID = 37;
    public static final int SHOOTER_KICKER_CAN_ID = 38;

    public static final int HOPPER_ROLLER_CAN_ID = 26;
  }

  public static class DIOPorts {}

  public enum ControlMode {
    POSITION,
    VELOCITY,
    VOLTAGE
  }

  public static class ArmConstants {
    public static final double SUPPLY_CURRENT_LIMIT = 40.0;
    public static final double STATOR_CURRENT_LIMIT = 80.0;

    public static final double GEAR_RATIO =
        RobotMode.ROBOT == RobotType.ROBOT_2024_RETIRED_MAESTRO
            ? (50.0 / 12.0) * (50.0 / 20.0) * (42.0 / 18.0)
            : 50.07;

    public static final double MOI = 0.00001;

    public static final Rotation2d MAX_ARM_ANGLE = Rotation2d.fromDegrees(165);
    public static final Rotation2d MIN_ARM_ANGLE = Rotation2d.fromDegrees(-90);
    public static final Rotation2d HOME_POSITION = Rotation2d.fromDegrees(153.808);

    public static final Rotation2d ARM_SAFE_ANGLE = Rotation2d.fromDegrees(-87);

    public static final Distance ARM_LENGTH =
        RobotMode.ROBOT == RobotType.ROBOT_2024_RETIRED_MAESTRO
            ? Units.Inches.of(14)
            : Units.Inches.of(22.080109);

    public static final String ROOT_TABLE = "Arm";
  }

  public static class ClimberConstants {
    public static final double SUPPLY_CURRENT_LIMIT = 60.0;
    public static final double STATOR_CURRENT_LIMIT = 90.0;

    public static final double WINCH_GEAR_RATIO = (50.0 / 12.0) * (50.0 / 20.0) * (42.0 / 18.0);

    public static final double WINCH_MOI = 0.15;

    public static final Rotation2d MAX_CLIMBER_ANGLE = Rotation2d.fromRotations(12.0);
    public static final Rotation2d MIN_CLIMBER_ANGLE = Rotation2d.fromRotations(0);
    public static final Rotation2d HOME_POSITION = MAX_CLIMBER_ANGLE;

    public static final Rotation2d CLIMBER_SAFE_ANGLE = Rotation2d.fromDegrees(-87);

    public static final Distance CLIMBER_LENGTH = Units.Inches.of(14);

    public static final Rotation2d SERVO_LOCK_ANGLE = Rotation2d.fromDegrees(90);

    public static final Rotation2d SERVO_UNLOCK_ANGLE = Rotation2d.fromDegrees(170);

    public static final String SERO_ROOT_TABLE = "ClimberServo";

    public static final Rotation2d OUT_WINCH_ROTATIONS = Rotation2d.fromRotations(6);

    public static final Rotation2d IN_WINCH_ROTATIONS = Rotation2d.fromRotations(0);

    public static final Rotation2d CLIMB_WINCH_ROTATIONS = Rotation2d.fromRotations(3);

    public static final String WINCH_ROOT_TABLE = "ClimberWinch";
  }

  public static class VisionGamepieceConstants {
    public static final Pose3d GAMEPIECE_CAMERA_POSE =
        new Pose3d(
            Units.Inches.of(5.3).in(Units.Meters),
            0.0,
            Units.Inches.of(18.725).in(Units.Meters),
            new Rotation3d(0.0, Math.toRadians(-12), 0.0));
    public static final String CAMERA_NAME = RobotConfig2024.OBJECT_DETECTION_CAMERA_NAME;
    public static final int RESOLUTION_WIDTH_PIXELS = 960;
    public static final int RESOLUTION_HEIGHT_PIXELS = 720;
    public static final Rotation2d FOV_DIAGONAL = Rotation2d.fromDegrees(128.2);
    public static final double AVERAGE_ERROR_PIXELS = 0.35;
    public static final double AVERAGE_STANDARD_DEVIATION_PIXELS = 0.1;
    public static final double FPS = 15;
    public static final double AVERAGE_LATENCY_MS = 25;
    public static final double AVERAGE_STANDARD_DEVIATION_MS = 10;
  }

  public static class AutoConstants {
    public static final Distance DIST_TO_START_INTAKING = Units.Meters.of(1.0);
  }

public static class ShooterConstants {

    public static final Distance SHOOTER_LENGTH = Units.Inches.of(12.0);

    public static class Hood {
      public static final double GEARING = 125.0;

      public static final Rotation2d MIN_ANGLE_RAD = Rotation2d.fromDegrees(12.0);
      public static final Rotation2d MAX_ANGLE_RAD = Rotation2d.fromDegrees(59);
      public static final Rotation2d IDLE_ANGLE_RAD = Rotation2d.fromDegrees(30);

      public static final double SIM_INITIAL_ANGLE = Math.toRadians(14.0);
      public static final Distance MIN_DISTANCE = Units.Inches.of(40.0);

      public static final double SUPPLY_CURRENT_LIMIT = 40;
      public static final double STATOR_CURRENT_LIMIT = 40;

      public static final double CORRECT_ANGLE_RANGE = 2;
    }

    public static class Flywheel {
      public static final double MOI = 5.0;
      public static final double GEARING = (18.0 / 30.0);
      public static final double SUPPLY_CURRENT_LIMIT = 40;
      public static final double STATOR_CURRENT_LIMIT = 40;
      public static final double PEAK_REVERSE_VOLTAGE = 0;
      public static final double SUPPLY_VOLTAGE_TIME = 0.02;
      public static final double IDLE_FLYWHEEL_RPM = 2000;
      public static final double CORRECT_RPM_RANGE = 50;
    }

    public static class Kicker {
      public static final double MOI = 5.0;
      public static final double GEARING = 1.0;
      public static final double SUPPLY_CURRENT_LIMIT = 40;
      public static final double STATOR_CURRENT_LIMIT = 40;
      public static final double SENSOR_TO_MECHANISM_RATIO = 1.42;
      public static final double SHOOTING_KICKER_RPM = 100;
    }

    public static class Interpolation {
      public static final InterpolatingDoubleTreeMap HOOD_ANGLE_FOR_HUB_DISTANCE =
          new InterpolatingDoubleTreeMap();
      public static final InterpolatingDoubleTreeMap RPM_FOR_HUB_DISTANCE =
          new InterpolatingDoubleTreeMap();

      public Interpolation() {

        HOOD_ANGLE_FOR_HUB_DISTANCE.put(1.0, 61.80);
        HOOD_ANGLE_FOR_HUB_DISTANCE.put(1.5, 51.50);
        HOOD_ANGLE_FOR_HUB_DISTANCE.put(2.0, 42.75);
        HOOD_ANGLE_FOR_HUB_DISTANCE.put(2.5, 36.70);
        HOOD_ANGLE_FOR_HUB_DISTANCE.put(3.0, 34.00);
        HOOD_ANGLE_FOR_HUB_DISTANCE.put(3.5, 31.20);
        HOOD_ANGLE_FOR_HUB_DISTANCE.put(4.0, 29.50);
        HOOD_ANGLE_FOR_HUB_DISTANCE.put(4.5, 27.50);
        HOOD_ANGLE_FOR_HUB_DISTANCE.put(5.0, 26.00);
        HOOD_ANGLE_FOR_HUB_DISTANCE.put(5.5, 24.85);
        HOOD_ANGLE_FOR_HUB_DISTANCE.put(6.0, 23.60);
        HOOD_ANGLE_FOR_HUB_DISTANCE.put(6.5, 22.80);

        RPM_FOR_HUB_DISTANCE.put(1.0, 2000.0);
        RPM_FOR_HUB_DISTANCE.put(2.0, 3000.0);
        RPM_FOR_HUB_DISTANCE.put(3.0, 4500.0);
        RPM_FOR_HUB_DISTANCE.put(4.0, 5200.0);
        RPM_FOR_HUB_DISTANCE.put(7.0, 6000.0);
        RPM_FOR_HUB_DISTANCE.put(8.0, 6800.0);
        RPM_FOR_HUB_DISTANCE.put(9.5, 7200.0);
        RPM_FOR_HUB_DISTANCE.put(11.5, 8000.0);
      }
    }
  }

  public static volatile boolean unusedCode = false;
}
