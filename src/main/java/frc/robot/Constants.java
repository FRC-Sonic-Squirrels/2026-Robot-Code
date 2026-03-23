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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.team2930.AllianceAwareTranslation2d;
import frc.lib.team2930.TunableInterpolationMap;
import frc.robot.Constants.RobotMode.RobotType;
import frc.robot.configs.*;

import java.util.HashMap;
import java.util.function.Supplier;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double        kDefaultPeriod    = 0.02;
  public static final Translation2d zeroTranslation2d = new Translation2d();
  public static final Rotation2d    zeroRotation2d    = new Rotation2d();
  public static final Pose2d        zeroPose2d        = new Pose2d();
  public static final Pose3d        zeroPose3d        = new Pose3d();

  public static boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
  }

  public static class RobotMode {
    private static final RobotType ROBOT = RobotType.ROBOT_2026_DOUG;

    public static boolean isSimBot() {
      return getRobot().isSimulator;
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
      // @formatter:off
      ROBOT_SIMBOT(SimulatorRobotConfig::new, true),
      ROBOT_SIMBOT_REAL_CAMERAS(SimulatorRobotConfig::new, true),
      ROBOT_2026_DOUG(RobotConfig2026::new, false); // Someone please name it...
      // @formatter:on

      public final Supplier<RobotConfig> config;
      public final boolean               isSimulator;

      RobotType(Supplier<RobotConfig> config, boolean isSimulator) {
        this.config      = config;
        this.isSimulator = isSimulator;
      }
    }

    public enum Mode {
      REAL, REPLAY, SIM
    }
  }

  public static double MAX_VOLTAGE = 12.0;

  public static class RobotDimensions {
    public static Distance BUMPER_THICKNESS = Units.Inches.of(4.1);

    /**
     * x = left to right
     *
     * <p>
     * y = front to back
     */
    public static Translation2d ROBOT_DIMENSIONS_WITHOUT_BUMPERS() {
      if (RobotMode.getRobot() == RobotType.ROBOT_2026_DOUG) {
        return new Translation2d(Units.Inches.of(27.0), Units.Inches.of(32.5));
      } else {
        return new Translation2d(Units.Inches.of(28.5), Units.Inches.of(30.5));
      }
    }

    /**
     * x = left to right
     *
     * <p>
     * y = front to back
     */
    public static Translation2d ROBOT_DIMENSIONS_WITH_BUMPERS = ROBOT_DIMENSIONS_WITHOUT_BUMPERS()
        .plus(new Translation2d(BUMPER_THICKNESS.times(2.0), BUMPER_THICKNESS.times(2.0)));
  }

  public static class FieldConstants {
    // Official Field dimensions from field dimensions manual
    public static Distance FIELD_LENGTH        = Units.Inches.of(650.12);
    public static Distance FIELD_WIDTH         = Units.Inches.of(316.64);
    public static double   FIELD_LENGTH_METERS = FIELD_LENGTH.in(Units.Meters);
    public static double   FIELD_WIDTH_METERS  = FIELD_WIDTH.in(Units.Meters);

    public static Distance HUB_DIST_FROM_WALL = Units.Inches.of(156.61);

    public static Distance HUB_WIDTH  = Units.Inches.of(41.73);
    public static Distance HUB_LENGTH = Units.Inches.of(47.00);

    public static Distance HUB_DIAGONAL_WIDTH = HUB_WIDTH.times(2.0 / Math.sqrt(3));

    public static Distance ALLIANCE_ZONE_LENGTH = Units.Inches.of(182.11);

    public static AllianceAwareTranslation2d HUB_CENTER_POSE = new AllianceAwareTranslation2d(
        new Translation2d(HUB_DIST_FROM_WALL.plus(HUB_WIDTH.div(2.0)), FIELD_WIDTH.div(2.0)));

    public static AllianceAwareTranslation2d RIGHT_PASS_AIM_POINT = new AllianceAwareTranslation2d(
        new Translation2d(FIELD_LENGTH.div(4.0), FIELD_WIDTH.times(1.0 / 4.0)));
    public static AllianceAwareTranslation2d LEFT_PASS_AIM_POINT  = new AllianceAwareTranslation2d(
        new Translation2d(FIELD_LENGTH.div(4.0), FIELD_WIDTH.times(3.0 / 4.0)));

    public static AllianceAwareTranslation2d[] PASS_AIM_POINTS = {
        RIGHT_PASS_AIM_POINT,
        LEFT_PASS_AIM_POINT
    };
  }

  public static class MotorConstants {
    public static class KrakenConstants {
      public static final double MAX_RPM                    = 6000.0;
      public static final double NOMINAL_VOLTAGE_VOLTS      = 12.0;
      public static final double STALL_TORQUE_NEWTON_METERS = 7.09;
      public static final double STALL_CURRENT_AMPS         = 40.0;
      public static final double FREE_CURRENT_AMPS          = 30.0;
      public static final double FREE_SPEED_RPM             = 6000.0;
      public static final double SUPPLY_VOLTAGE_TIME        = 0.02;
    }
  }

  public static class IntakeConstants {
    public static class RollerConstants {

      public static final double ROLLER_GEARING = 1.0;
      public static final double MOI            = 0.05;

      public static final double SUPPLY_CURRENT_LIMIT = 60.0;
      public static final double STATOR_CURRENT_LIMIT = 60.0;

      public static final String ROOT_TABLE = "Rollers";
    }

    public static class PivotConstants {
      public static final double HIGH_SUPPLY_CURRENT_LIMIT = 60;  // 20.0;
      public static final double HIGH_STATOR_CURRENT_LIMIT = 60;  // 30.0;
      public static final double LOW_SUPPLY_CURRENT_LIMIT  = 10.0;
      public static final double LOW_STATOR_CURRENT_LIMIT  = 10.0;

      public static final double GEAR_RATIO = 25.0043;

      public static final double MOI = .00001;

      public static final Rotation2d MAX_PIVOT_ANGLE_LEFT  = Rotation2d.fromDegrees(63.5);
      public static final Rotation2d MAX_PIVOT_ANGLE_RIGHT = Rotation2d.fromDegrees(63.5);
      public static final Rotation2d MIN_PIVOT_ANGLE       = Rotation2d.fromDegrees(0);
      public static final Rotation2d HOME_POSITION         = Rotation2d.fromDegrees(0);
      public static final Rotation2d ZERO_POSITION         = Rotation2d.fromDegrees(0);

      public static final Rotation2d PASSOFF_PIVOT_ANGLE = Rotation2d.fromDegrees(92.5);

      public static final Rotation2d INITIAL_PIVOT_ANGLE = Rotation2d.fromDegrees(120);
      public static final Rotation2d PIVOT_STOWED_ANGLE  = Rotation2d.fromDegrees(105);

      public static final Rotation2d PIVOT_SAFE_ANGLE = Rotation2d.fromDegrees(90);

      public static final Rotation2d INTERNAL_ANGLE    = Rotation2d.fromDegrees(0);
      public static final Rotation2d EXTERNAL_ANGLE    = Rotation2d.fromDegrees(180);
      public static final Rotation2d ALGAE_SCORE_ANGLE = Rotation2d.fromDegrees(45);

      public static final Distance PIVOT_LENGTH = Units.Inches.of(6.215);
    }
  }

  public static class HopperConstants {
    public static class RollerConstants {

      public static final double ROLLER_GEARING = 4.44444;
      public static final double MOI            = 0.05;

      public static final double SUPPLY_CURRENT_LIMIT = 10;
      public static final double STATOR_CURRENT_LIMIT = 10;

      public static final String ROLLER                   = "Rollers";
      public static final int    SHOOTING_ROLLER_VELOCITY = 2000;
    }
  }

  public static class LEDConstants {
    public static final int PWM_PORT       = 9;
    public static final int MAX_LED_LENGTH = 60;
    public static final int LED_BUFFER     = 36;
  }

  public static class DIOPorts {
    public static final int SHOOTER_BEAM_BREAKER_DOI_ID = 8;
  }

  public enum ControlMode {
    POSITION, VELOCITY, VOLTAGE
  }

  public static class ArmConstants {
    public static final Distance ARM_LENGTH = RobotMode.ROBOT == RobotType.ROBOT_2026_DOUG ? Units.Inches.of(14)
        : Units.Inches.of(22.080109);
  }

  public static class ClimberConstants {
    // TODO make all climber constants accurate
    public static final double SUPPLY_CURRENT_LIMIT = 60.0;
    public static final double STATOR_CURRENT_LIMIT = 90.0;

    public static final double FOLLOWER_CLIMBER_GEAR_RATIO = 100;
    public static final double LEADER_CLIMBER_GEAR_RATIO   = 100;

    public static final double CLIMBER_MOI = 0.15;

    public static final boolean HAS_FOLLOWER_CLIMBER = false;

    // in meters, but converted to inches in the sim
    public static final double MAX_FOLLOWER_CLIMBER_HEIGHT = 0.6858;
    public static final double MAX_LEADER_CLIMBER_HEIGHT   = 0.6858;
    public static final double MIN_FOLLOWER_CLIMBER_HEIGHT = 0;
    public static final double MIN_LEADER_CLIMBER_HEIGHT   = 0;

    public static final double   GEAR_RATIO      = 6.4935;
    public static final Distance PULLEY_DIAMETER = Units.Inches.of(2.35);
    public static final double   CARRIAGE_MASS   = 2.0;                  // arbitrary

    public static final double INCHES_TO_MOTOR_ROT = GEAR_RATIO / (Math.PI * PULLEY_DIAMETER.in(Units.Inches));

    public static final Distance CLIMBER_LENGTH           = Units.Inches.of(14);
    public static final Distance FOLLOWER_PULLEY_DIAMETER = Units.Inches.of(1.0);
    public static final Distance LEADER_PULLEY_DIAMETER   = Units.Inches.of(1.0);
    public static final double   FOLLOWER_CARRIAGE_MASS   = 50.0;
    public static final double   LEADER_CARRIAGE_MASS     = 50.0;

    public static final int SERVO_CHANNEL = 8;

    public static final Rotation2d SERVO_LOCK_ANGLE = Rotation2d.fromDegrees(90);

    public static final Rotation2d SERVO_UNLOCK_ANGLE = Rotation2d.fromDegrees(170);

    public static final Distance MAX_HEIGHT = Units.Inches.of(27);

    public static final Distance LEADER_HEIGHT_CLIMBL1 = Units.Inches.of(25);
    public static final Distance LEADER_HEIGHT_CLIMBL2 = Units.Inches.of(24);
    public static final Distance LEADER_HEIGHT_CLIMBL3 = Units.Inches.of(23);

    public static final Distance FOLLOWER_HEIGHT_EXTENDED = Units.Inches.of(18);

    public static final Distance LEADER_HEIGHT_RETRACTED   = Units.Inches.of(5);
    public static final Distance FOLLOWER_HEIGHT_RETRACTED = Units.Inches.of(5);

    public static final Distance LEADER_HEIGHT_STOWED   = Units.Inches.of(0);
    public static final Distance FOLLOWER_HEIGHT_STOWED = Units.Inches.of(0);
  }

  public static class ShooterConstants {

    public static final Distance SHOOTER_LENGTH = Units.Inches.of(12.0);

    public static class Hood {

      public static final double MOI = 5.0;

      public static final double GEARING = 8.3333 * 12;

      public static final Rotation2d MIN_ANGLE_RAD  = Rotation2d.fromDegrees(12);
      public static final Rotation2d MAX_ANGLE_RAD  = Rotation2d.fromDegrees(12 + 20);
      public static final Rotation2d IDLE_ANGLE_RAD = Rotation2d.fromDegrees(12);

      public static final double SIM_INITIAL_ANGLE = Math.toRadians(14.0);

      public static final double SUPPLY_CURRENT_LIMIT = 15;
      public static final double STATOR_CURRENT_LIMIT = 15;

      public static final double CORRECT_ANGLE_RANGE = 2;
    }

    public static class Flywheel {
      public static final double MOI                  = 5.0;
      public static final double GEARING              = 0.6667;
      public static final double SUPPLY_CURRENT_LIMIT = 40;
      public static final double STATOR_CURRENT_LIMIT = 40;
      public static final double PEAK_REVERSE_VOLTAGE = 0;
      public static final double SUPPLY_VOLTAGE_TIME  = 0.02;
      public static final double IDLE_FLYWHEEL_RPM    = 3000;  // 2000;
      public static final double DEBOUNCE_AT_TARGET   = 0.15;
    }

    public static class Kicker {
      public static final double MOI                                  = 5.0;
      public static final double GEARING                              = 2.25;
      public static final double SUPPLY_CURRENT_LIMIT                 = 60;
      public static final double STATOR_CURRENT_LIMIT                 = 60;
      public static final double SHOOTING_KICKER_RPM                  = 4000;
      public static final double UNJAM_KICKER_RPM                     = -1200;
      public static final double STALL_CURRENT_THRESHOLD              = 40;
      public static final double PRELOADING_KICKER_RPM                = 800;
      public static final double DURATION_TO_CONFIRM_STALL            = 0.2;
      public static final double DURATION_TO_CONFIRM_UNSTALL          = 0.4;
      public static final double DURATION_TO_CONFIRM_PERSISTANT_STALL = 1;
    }

    public static class Interpolation {
      public static TunableInterpolationMap generateHoodAngleForHubDistanceMap() {
        var map = new HashMap<Double, Double>();
        map.put(1.0, 12.0);
        map.put(1.5, 16.0);
        map.put(2.0, 18.0);
        map.put(2.5, 20.0);
        map.put(3.0, 20.0);
        map.put(3.5, 20.0);
        map.put(4.0, 20.0);
        map.put(4.5, 22.0);
        map.put(5.0, 24.0);
        return new TunableInterpolationMap("HOOD_ANGLE_FOR_HUB_DISTANCE", map);
      }

      public static TunableInterpolationMap generateFlywheelRpmForHubDistanceMap() {
        var map = new HashMap<Double, Double>();
        map.put(1.0, 3000.0);
        map.put(1.5, 3100.0);
        map.put(2.0, 3200.0);
        map.put(2.5, 3500.0);
        map.put(3.0, 3700.0);
        map.put(3.5, 4000.0);
        map.put(4.0, 4300.0);
        map.put(4.5, 4700.0);
        map.put(5.0, 5200.0);
        return new TunableInterpolationMap("RPM_FOR_HUB_DISTANCE", map);
      }

      public static TunableInterpolationMap generateHoodAngleForAllianceZoneDistanceMap() {
        var map = new HashMap<Double, Double>();
        map.put(1.0, 30.0);
        map.put(1.5, 30.0);
        map.put(2.0, 30.0);
        map.put(2.5, 30.0);
        map.put(3.0, 30.0);
        map.put(3.5, 30.0);
        map.put(4.0, 30.0);
        map.put(4.5, 30.0);
        map.put(5.0, 30.0);
        map.put(5.5, 30.0);
        map.put(6.0, 30.0);
        map.put(6.5, 30.0);
        return new TunableInterpolationMap("HOOD_ANGLE_FOR_ALLIANCE_ZONE_DISTANCE", map);
      }

      public static TunableInterpolationMap generateFlywheelRpmForAllianceZoneDistanceMap() {
        var map = new HashMap<Double, Double>();
        map.put(1.0, 2000.0);
        map.put(2.0, 3000.0);
        map.put(3.0, 4500.0);
        map.put(4.0, 5200.0);
        map.put(7.0, 6000.0);
        map.put(8.0, 6800.0);
        map.put(9.5, 7200.0);
        map.put(11.5, 8000.0);
        return new TunableInterpolationMap("RPM_FOR_ALLIANCE_ZONE_DISTANCE", map);
      }
    }
  }

  public static volatile boolean unusedCode = false;
}
