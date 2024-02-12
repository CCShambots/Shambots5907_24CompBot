package frc.robot;

import static com.ctre.phoenix.led.LarsonAnimation.BounceMode.Front;
import static frc.robot.Constants.Lights.Hardware.NUM_LIGHTS;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.ShamLib.Candle.RGB;
import frc.robot.ShamLib.PIDGains;
import frc.robot.ShamLib.ShamLibConstants;
import frc.robot.ShamLib.motors.talonfx.PIDSVGains;
import frc.robot.ShamLib.motors.tuning.LoggedTunablePIDSV;
import frc.robot.ShamLib.swerve.SwerveSpeedLimits;
import frc.robot.ShamLib.swerve.module.ModuleInfo;
import java.util.Optional;
import java.util.function.UnaryOperator;

public class Constants {
  public static ShamLibConstants.BuildMode currentBuildMode = ShamLibConstants.BuildMode.SIM;
  public static final CurrentLimitsConfigs DEFAULT_CURRENT_LIMIT =
      new CurrentLimitsConfigs().withSupplyCurrentLimit(20).withSupplyCurrentLimitEnable(true);

  public static final boolean ALLOW_TUNING = true;

  public static final class Controller {
    public static final int LEFT_FLIGHT_STICK_ID = 1;
    public static final int RIGHT_FLIGHT_STICK_ID = 2;
    public static final int OPERATOR_CONTROLLER_ID = 0;

    public static final UnaryOperator<Double> DRIVE_CONVERSION =
        (input) -> (Math.copySign(input * input, input));
    public static final double DEADBAND = 0.025;
  }

  public static final class PhysicalConstants {
    // METERS

    public static final Pose3d CHASSIS_TO_SHOOTER =
        new Pose3d(
            new Translation3d(Units.inchesToMeters(-2.999766), 0, Units.inchesToMeters(9.434239)),
            new Rotation3d(Math.toRadians(90), 0, Math.toRadians(180)));

    // how much taller climber is than shooter pivot when stowed
    public static double CLIMBER_Y_DISTANCE_FROM_SHOOTER_PIVOT = 0.0;

    // how far away climber is from shooter pivot on front/back axis
    public static double CLIMBER_X_DISTANCE_FROM_SHOOTER_PIVOT = 0.0;

    public static Pose2d BLUE_SPEAKER =
        new Pose2d(new Translation2d(-0.039243, 5.557), Rotation2d.fromDegrees(0));
    public static Pose2d BLUE_AMP =
        new Pose2d(new Translation2d(1.799, 0.0218), Rotation2d.fromDegrees(-90));
    public static Pose2d BLUE_CENTER_TRAP =
        new Pose2d(new Translation2d(5.286, 4.115), Rotation2d.fromDegrees(0));
    public static Pose2d BLUE_LEFT_TRAP =
        new Pose2d(new Translation2d(4.597, 4.513), Rotation2d.fromDegrees(120));
    public static Pose2d BLUE_RIGHT_TRAP =
        new Pose2d(new Translation2d(15.652, 4.512), Rotation2d.fromDegrees(-120));

    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT;

    static {
      try {
        APRIL_TAG_FIELD_LAYOUT =
            AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      } catch (Exception e) {
        throw new RuntimeException("Could not load AprilTag field layout from WPI");
      }
    }
  }

  public static final class Vision {
    public static final class Sim {}

    public static final class Hardware {}

    public static final class Settings {
      public static final int LIMELIGHT_NOTE_TRACK_PIPELINE = 0;
    }
  }

  public static final class Shooter {
    public static final class Sim {}

    public static final class Hardware {}

    public static final class Settings {

      public static final InterpolatingDoubleTreeMap FLYWHEEL_SPEAKER_DISTANCE_LUT =
          new InterpolatingDoubleTreeMap();

      public static final InterpolatingDoubleTreeMap ARM_SPEAKER_DISTANCE_OFFSET_LUT =
          new InterpolatingDoubleTreeMap();

      public static final InterpolatingDoubleTreeMap FLYWHEEL_TRAP_DISTANCE_LUT =
          new InterpolatingDoubleTreeMap();
      public static final InterpolatingDoubleTreeMap ARM_TRAP_DISTANCE_LUT =
          new InterpolatingDoubleTreeMap();

      public static final double SPEAKER_TARGET_HEIGHT = 2.05;
      public static final double TRAP_TARGET_HEIGHT = 1.52;

      static {
        // FLYWHEEL SPEAKER VALUES
        FLYWHEEL_SPEAKER_DISTANCE_LUT.put(0.0, Flywheel.Settings.BASE_SHOT_VELOCITY);
        FLYWHEEL_SPEAKER_DISTANCE_LUT.put(20.0, Flywheel.Settings.BASE_SHOT_VELOCITY);

        // ARM SPEAKER OFFSETS
        ARM_SPEAKER_DISTANCE_OFFSET_LUT.put(0.0, 0.0);
        ARM_SPEAKER_DISTANCE_OFFSET_LUT.put(20.0, 0.0);

        // FLYWHEEL TRAP VALUES
        FLYWHEEL_TRAP_DISTANCE_LUT.put(0.0, Flywheel.Settings.BASE_SHOT_VELOCITY);
        FLYWHEEL_TRAP_DISTANCE_LUT.put(20.0, Flywheel.Settings.BASE_SHOT_VELOCITY);

        // ARM TRAP OFFSETS
        ARM_TRAP_DISTANCE_LUT.put(0.0, 0.0);
        ARM_TRAP_DISTANCE_LUT.put(20.0, 0.0);
      }
    }
  }

  public static final class Arm {
    public static final class Sim {
      public static final double LEADER_INERTIA = 0.001;
      public static final double FOLLOWER_INERTIA = 0.001;
    }

    public static final class Hardware {
      public static final int LEADER_ID = 20;
      public static final int FOLLOWER_ID = 21;
      public static final int ENCODER_ID = 3;

      public static final double ENCODER_RATIO = (10.0 / 58.0) * 2 * Math.PI;
      public static final double MOTOR_RATIO =
          (10.0 / 64.0) * (18.0 / 50.0) * (10.0 / 58.0) * 2 * Math.PI;

      public static final boolean ENCODER_INVERTED = true;
      public static final boolean LEADER_INVERTED = false;
      public static final boolean FOLLOWER_INVERTED = false;

      public static final double ENCODER_OFFSET = 0.0;

      public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;

      public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = DEFAULT_CURRENT_LIMIT;

      public static final LoggedTunablePIDSV GAINS =
          new LoggedTunablePIDSV(
              "Shooter Arm Gains", new PIDSVGains(0, 0, 0, 0, 0), () -> ALLOW_TUNING);
    }

    public static final class Settings {
      public static final double VELOCITY = (2 / 60.0) * 2 * Math.PI; // RAD/s
      public static final double ACCELERATION = (4 / 60.0) * 2 * Math.PI; // RAD/s/s
      public static final double JERK = 10_000; // RAD/s/s/s

      public static final double POSITION_READY_TOLERANCE = 5 * (Math.PI / 180); // RAD

      public static final double BASE_SHOT_POSITION = 65 * (Math.PI / 180); // RAD
      public static final double AMP_POSITION = 50 * (Math.PI / 180); // RAD
      public static final double TRAP_PREP_POSITION = 50 * (Math.PI / 180); // RAD
      public static final double FULL_STOW_POSITION = 30 * (Math.PI / 180); // RAD
      public static final double PARTIAL_STOW_POSITION = 40 * (Math.PI / 180); // RAD
      public static final double CHUTE_INTAKE_POSITION = 60 * (Math.PI / 180); // RAD

      public static final double AUTO_SYNC_TOLERANCE = 0.1;
      public static final double AUTO_SYNC_MAX_VELOCITY = 0.1; // RAD/s

      public static final boolean ENABLE_AUTO_SYNC = false;
      public static final double MIN_TIME_BETWEEN_SYNC = 2.0;

      public static final double VOLTAGE_INCREMENT = 0.25;

      public static final double MIN_ANGLE = 30.0 * (Math.PI / 180);
      public static final double MAX_ANGLE = 60 * (Math.PI / 180);
    }
  }

  public static final class Flywheel {
    public static final class Sim {
      public static final double TOP_INERTIA = 0.0001;
      public static final double BOTTOM_INERTIA = 0.0001;
    }

    public static final class Hardware {
      public static final int TOP_MOTOR_ID = 10;
      public static final int BOTTOM_MOTOR_ID = 11;

      public static final double TOP_MOTOR_RATIO = 1;
      public static final double BOTTOM_MOTOR_RATIO = 1;

      public static final boolean TOP_MOTOR_INVERTED = false;
      public static final boolean BOTTOM_MOTOR_INVERTED = false;

      public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;

      public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = DEFAULT_CURRENT_LIMIT;

      public static final LoggedTunablePIDSV GAINS =
          new LoggedTunablePIDSV(
              "Top Flywheel Gains", new PIDSVGains(0, 0, 0, 0, 0), () -> ALLOW_TUNING);

      public static final LoggedTunablePIDSV BOTTOM_MOTOR_GAINS =
          new LoggedTunablePIDSV(
              "Bottom Flywheel Gains", new PIDSVGains(0, 0, 0, 0, 0), () -> ALLOW_TUNING);
    }

    public static final class Settings {
      public static final double BASE_SHOT_VELOCITY = 5800 / 60.0; // RPS
      public static final double SPIN_UP_READY_TOLERANCE = 60 / 60.0; // RPS

      public static final double PASS_THROUGH_SPEED = 5 / 60.0; // RPS

      public static final double CHUTE_INTAKE_SPEED = 600 / 60.0; // RPS
      public static final double AMP_SPEED = 1000 / 60.0; // RPS

      public static final double VOLTAGE_INCREMENT = 0.25;
    }
  }

  public static final class Intake {
    public static final class Hardware {
      public static final int TOP_ID = 30;
      public static final int BOTTOM_ID = 31;
      public static final int PROX_ID = 0;

      public static final double TOP_RATIO = 1;
      public static final double BOTTOM_RATIO = 1;

      public static final boolean TOP_INVERTED = false;
      public static final boolean BOTTOM_INVERTED = false;

      public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;

      public static final CurrentLimitsConfigs CURRENT_LIMIT = DEFAULT_CURRENT_LIMIT;

      public static final LoggedTunablePIDSV TOP_GAINS =
          new LoggedTunablePIDSV(
              "Intake Belt Gains", new PIDSVGains(0, 0, 0, 0, 0), () -> ALLOW_TUNING);
    }

    public static final class Sim {
      public static final double BELT_INERTIA = 0.001;
    }

    public static final class Settings {
      public static final double BELT_SPEED = 2000 / 60.0; // RPS

      public static final double VOLTAGE_INC = 0.25;
    }
  }

  public static final class Climbers {
    public static final class Sim {
      public static final double INERTIA = 0.01;
    }

    public static final class Hardware {
      public static final int LEFT_CLIMBER_ID = 40;
      public static final int RIGHT_CLIMBER_ID = 41;

      public static final boolean LEFT_INVERTED = false;
      public static final boolean RIGHT_INVERTED = false;

      public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = DEFAULT_CURRENT_LIMIT;

      // rotations to meters
      public static final double CLIMBER_RATIO = 1;
    }

    public static final class Settings {
      public static final LoggedTunablePIDSV FREE_GAINS =
          new LoggedTunablePIDSV(
              "Climber Free Gains", new PIDSVGains(0, 0, 0, 0, 0), () -> ALLOW_TUNING);

      public static final LoggedTunablePIDSV LOADED_GAINS =
          new LoggedTunablePIDSV(
              "Climber Loaded Gains", new PIDSVGains(0, 0, 0, 0, 0), () -> ALLOW_TUNING);

      // all in meters
      public static double FREE_VELOCITY = 1;
      public static double FREE_ACCELERATION = 1;
      public static double FREE_JERK = 0;

      public static double LOADED_VELOCITY = 1;
      public static double LOADED_ACCELERATION = 1;
      public static double LOADED_JERK = 0;

      // meters
      public static double SETPOINT_TOLERANCE = 0.05;

      public static int FREE_SLOT = 0;
      public static int LOADED_SLOT = 1;

      public static double EXTENSION_SETPOINT = 0;

      public static double VOLTAGE_INCREMENT = 0.125;
    }
  }

  public static final class Indexer {
    public static final class Hardware {
      public static final int BELT_MOTOR_ID = 50;

      public static final double BELT_RATIO = 1.0;

      public static final boolean INVERT_BELT_MOTOR = false;

      public static final int PROX_1_ID = 1;
      public static final int PROX_2_ID = 2;
      public static final int PROX_3_ID = 3;

      public static final CurrentLimitsConfigs BELT_MOTOR_CURRENT_LIMIT = DEFAULT_CURRENT_LIMIT;
      public static final NeutralModeValue BELT_MOTOR_NEUTRAL_MODE = NeutralModeValue.Coast;

      public static final LoggedTunablePIDSV BELT_GAINS =
          new LoggedTunablePIDSV(
              "Indexer Belt Gains", new PIDSVGains(0, 0, 0, 0, 0), () -> ALLOW_TUNING);
    }

    public static final class Settings {
      // rps
      public static final double EXPECT_SPEED = 20;
      public static final double PASS_THROUGH_SPEED = 33;
      public static final double INDEX_SPEED = 20;
      public static final double FEED_SPEED = 33;

      // seconds
      public static final double INDEX_TIMEOUT = 2;

      public static final double VOLTAGE_INCREMENT = 0.125;
    }

    public static final class Sim {
      public static final double MOTOR_INERTIA = 0.0001;
    }
  }

  public static final class Lights {
    public static final class Hardware {
      public static final int CANDLE_ID = 0;

      public static final int NUM_LIGHTS = 200;

      public static final double BRIGHTNESS = 1.0;
    }

    public static final class Settings {
      public static final int NUM_LIGHTS_WITHOUT_CANDLE = NUM_LIGHTS - 8;

      public static final double BOUNCE_SPEED = 0.75;
      public static final double BLINK_SPEED = .075;

      public static final RGB NO_RING_RGB = new RGB(0, 0, 0);
      public static final RGB ERROR_RGB = new RGB(255, 0, 0);
      public static final RGB HOLDING_RING = new RGB(0, 0, 255);
      public static final RGB READY_TO_SHOOT = new RGB(0, 255, 0);

      public static final Animation DISABLED_ANIMATION =
          new LarsonAnimation(0, 0, 255, 0, BOUNCE_SPEED, NUM_LIGHTS_WITHOUT_CANDLE, Front, 7, 8);

      public static final Animation AUTO_ANIMATION =
          new ColorFlowAnimation(
              0, 0, 255, 0, .0125, NUM_LIGHTS, ColorFlowAnimation.Direction.Forward);

      public static final Animation TARGETING_ANIMATION =
          new StrobeAnimation(255, 255, 0, 0, BLINK_SPEED, NUM_LIGHTS);

      public static final Animation INTAKE_ANIMATION =
          new StrobeAnimation(0, 0, 255, 0, BLINK_SPEED, NUM_LIGHTS);

      public static final Animation EJECT_ANIMATION =
          new StrobeAnimation(255, 0, 0, 0, BLINK_SPEED, NUM_LIGHTS);

      public static final Animation AUTOMATIC_SCORE_ANIMATION =
          new TwinkleAnimation(
              0, 0, 255, 0, 0.5, NUM_LIGHTS, TwinkleAnimation.TwinklePercent.Percent76);
    }
  }

  public static final class Drivetrain {
    public static final class Sim {}

    public static final class Hardware {
      public static final int GYRO_ID = 60;

      public static final double TRACK_WIDTH = Units.inchesToMeters(24.75);
      public static final double WHEEL_BASE = Units.inchesToMeters(24.75);
      public static final double ROTATION_RADIUS =
          Math.sqrt(Math.pow(TRACK_WIDTH / 2.0, 2) + Math.pow(WHEEL_BASE / 2.0, 2)) * 2 * Math.PI;

      public static final String MODULE_CAN_BUS = "";
      public static final String GYRO_CAN_BUS = "";

      public static final ModuleInfo MODULE_1_INFO = // FRONT LEFT
          ModuleInfo.generateModuleInfo(
              ModuleInfo.SwerveModuleType.MK4i,
              ModuleInfo.SwerveModuleSpeedLevel.L3,
              60, // DRIVE MOTOR ID
              61, // TURN MOTOR ID
              60, // ENCODER ID
              0.0, // ENCODER OFFSET
              new Translation2d(
                  WHEEL_BASE / 2, TRACK_WIDTH / 2), // MODULE OFFSET FROM CENTER OF BOT
              false // DRIVE MOTOR INVERTED
              );

      public static final ModuleInfo MODULE_2_INFO = // BACK LEFT
          ModuleInfo.generateModuleInfo(
              ModuleInfo.SwerveModuleType.MK4i,
              ModuleInfo.SwerveModuleSpeedLevel.L3,
              62, // DRIVE MOTOR ID
              63, // TURN MOTOR ID
              61, // ENCODER ID
              0.0, // ENCODER OFFSET
              new Translation2d(
                  -WHEEL_BASE / 2, TRACK_WIDTH / 2), // MODULE OFFSET FROM CENTER OF BOT
              false // DRIVE MOTOR INVERTED
              );

      public static final ModuleInfo MODULE_3_INFO = // BACK RIGHT
          ModuleInfo.generateModuleInfo(
              ModuleInfo.SwerveModuleType.MK4i,
              ModuleInfo.SwerveModuleSpeedLevel.L3,
              64, // DRIVE MOTOR ID
              65, // TURN MOTOR ID
              62, // ENCODER ID
              0.0, // ENCODER OFFSET
              new Translation2d(
                  -WHEEL_BASE / 2, -TRACK_WIDTH / 2), // MODULE OFFSET FROM CENTER OF BOT
              false // DRIVE MOTOR INVERTED
              );

      public static final ModuleInfo MODULE_4_INFO = // FRONT RIGHT
          ModuleInfo.generateModuleInfo(
              ModuleInfo.SwerveModuleType.MK4i,
              ModuleInfo.SwerveModuleSpeedLevel.L3,
              66, // DRIVE MOTOR ID
              67, // TURN MOTOR ID
              63, // ENCODER ID
              0.0, // ENCODER OFFSET
              new Translation2d(
                  WHEEL_BASE / 2, -TRACK_WIDTH / 2), // MODULE OFFSET FROM CENTER OF BOT
              false // DRIVE MOTOR INVERTED
              );

      public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = DEFAULT_CURRENT_LIMIT;
    }

    public static final class Settings {
      public static final PIDGains AUTO_THETA_GAINS = new PIDGains(10, 0, 0);
      public static final PIDGains AUTO_TRANSLATION_GAINS = new PIDGains(6, 0, 0);

      public static final PIDSVGains MODULE_DRIVE_GAINS = new PIDSVGains(0, 0, 0, 0, 0);
      public static final PIDSVGains MODULE_TURN_GAINS = new PIDSVGains(0, 0, 0, 0, 0);

      public static final PIDGains HOLD_ANGLE_GAINS = new PIDGains(0, 0, 0);

      // m/s
      public static final double MAX_CHASSIS_SPEED = 5;
      public static final double MAX_CHASSIS_ACCELERATION = 10;
      public static final double MAX_CHASSIS_ROTATIONAL_SPEED =
          (MAX_CHASSIS_SPEED / Hardware.ROTATION_RADIUS) * (2 * Math.PI);
      public static final double MAX_CHASSIS_ROTATIONAL_ACCELERATION =
          MAX_CHASSIS_ROTATIONAL_SPEED * 3;
      public static final double MAX_MODULE_TURN_SPEED = 1000;
      public static final double MAX_MODULE_TURN_ACCELERATION = 1000;

      public static final Matrix<N3, N1> STATE_STD_DEVIATIONS =
          VecBuilder.fill(0.003, 0.003, 0.0002);

      // meters and radians
      public static final SwerveSpeedLimits PATH_FIND_SPEED =
          new SwerveSpeedLimits(
              MAX_CHASSIS_SPEED / 2,
              MAX_CHASSIS_ACCELERATION,
              MAX_CHASSIS_ROTATIONAL_SPEED / 2,
              MAX_CHASSIS_ROTATIONAL_ACCELERATION);
      public static final SwerveSpeedLimits TRAVERSE_SPEED =
          new SwerveSpeedLimits(
              MAX_CHASSIS_SPEED,
              MAX_CHASSIS_ACCELERATION,
              MAX_CHASSIS_ROTATIONAL_SPEED,
              MAX_CHASSIS_ROTATIONAL_ACCELERATION);
      public static final SwerveSpeedLimits AMP_SPEED =
          new SwerveSpeedLimits(
              MAX_CHASSIS_SPEED / 1.25,
              MAX_CHASSIS_ACCELERATION,
              MAX_CHASSIS_ROTATIONAL_SPEED / 1.25,
              MAX_CHASSIS_ROTATIONAL_ACCELERATION);
      public static final SwerveSpeedLimits INTAKE_SPEED =
          new SwerveSpeedLimits(
              MAX_CHASSIS_SPEED / 1.25,
              MAX_CHASSIS_ACCELERATION,
              MAX_CHASSIS_ROTATIONAL_SPEED / 1.25,
              MAX_CHASSIS_ROTATIONAL_ACCELERATION);
      public static final SwerveSpeedLimits SPEAKER_SPEED =
          new SwerveSpeedLimits(
              MAX_CHASSIS_SPEED / 1.25,
              MAX_CHASSIS_ACCELERATION,
              MAX_CHASSIS_ROTATIONAL_SPEED / 1.25,
              MAX_CHASSIS_ROTATIONAL_ACCELERATION);
      public static final SwerveSpeedLimits HUMAN_PLAYER_PICKUP_SPEED =
          new SwerveSpeedLimits(
              MAX_CHASSIS_SPEED / 1.75,
              MAX_CHASSIS_ACCELERATION,
              MAX_CHASSIS_ROTATIONAL_SPEED / 1.75,
              MAX_CHASSIS_ROTATIONAL_ACCELERATION);
      public static final SwerveSpeedLimits TRAP_SPEED =
          new SwerveSpeedLimits(
              MAX_CHASSIS_SPEED / 1.25,
              MAX_CHASSIS_ACCELERATION,
              MAX_CHASSIS_ROTATIONAL_SPEED / 1.25,
              MAX_CHASSIS_ROTATIONAL_ACCELERATION);

      public static final SwerveModuleState[] X_SHAPE =
          new SwerveModuleState[] {
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45))
          };

      // meters
      public static final double AMP_ROTATIONAL_DELAY = 0;
      public static final double CLIMB_ROTATION_DELAY = 0;
      public static final double HUMAN_PLAYER_SCORE_ROTATIONAL_DELAY = 0;

      // radians
      public static final double FACE_ANGLE_TOLERANCE = 0.02;

      public static final double TURN_VOLTAGE_INCREMENT = 0.125;
      public static final double DRIVE_VOLTAGE_INCREMENT = 0.125;
    }
  }

  public static DriverStation.Alliance alliance = DriverStation.Alliance.Red;
  public static boolean overrideAlliance = false;

  public static void applyAlliance(Optional<DriverStation.Alliance> newAlliance) {
    if (!overrideAlliance && newAlliance.isPresent()) {
      alliance = newAlliance.get();
    }
  }

  public static boolean FMSConnected() {
    return NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAlliance").exists();
  }

  public static boolean doubleEqual(double a, double b, double accuracy) {
    return Math.abs(a - b) < accuracy;
  }

  public static boolean doubleEqual(double a, double b) {
    return doubleEqual(a, b, 0.00001); // TODO: idk if this is fine or not
  }

  public static Pose2d mirror(Pose2d pose) {
    return new Pose2d(
        new Translation2d(
            PhysicalConstants.APRIL_TAG_FIELD_LAYOUT.getFieldLength() - pose.getX(),
            PhysicalConstants.APRIL_TAG_FIELD_LAYOUT.getFieldWidth() - pose.getY()),
        new Rotation2d(Math.PI).minus(pose.getRotation()));
  }

  public static Rotation2d rotationBetween(Pose2d pose1, Pose2d pose2) {
    return Rotation2d.fromRadians(
        Math.atan2(pose2.getY() - pose1.getY(), pose2.getX() - pose1.getX()));
  }
}
