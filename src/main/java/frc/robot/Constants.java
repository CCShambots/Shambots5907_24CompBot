package frc.robot;

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
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.ShamLib.PIDGains;
import frc.robot.ShamLib.ShamLibConstants;
import frc.robot.ShamLib.motors.talonfx.PIDSVGains;
import frc.robot.ShamLib.motors.tuning.LoggedTunablePIDSV;
import frc.robot.ShamLib.swerve.SwerveSpeedLimits;
import frc.robot.ShamLib.swerve.module.ModuleInfo;
import java.util.Map;
import java.util.NavigableMap;
import java.util.Optional;
import java.util.TreeMap;
import java.util.function.UnaryOperator;

public class Constants {
  public static ShamLibConstants.BuildMode currentBuildMode = ShamLibConstants.BuildMode.SIM;
  public static final CurrentLimitsConfigs DEFAULT_CURRENT_LIMIT =
      new CurrentLimitsConfigs().withSupplyCurrentLimit(20).withSupplyCurrentLimitEnable(true);

  public static final boolean ALLOW_TUNING = true;

  public static final class Controller {
    public static final int LEFT_FLIGHT_STICK_ID = 0;
    public static final int RIGHT_FLIGHT_STICK_ID = 0;
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

    public static Pose2d BLUE_SPEAKER = new Pose2d(new Translation2d(), new Rotation2d());
    public static Pose2d BLUE_AMP = new Pose2d(new Translation2d(), new Rotation2d());
    public static Pose2d BLUE_CENTER_TRAP = new Pose2d(new Translation2d(), new Rotation2d());
    public static Pose2d BLUE_LEFT_TRAP = new Pose2d(new Translation2d(), new Rotation2d());
    public static Pose2d BLUE_RIGHT_TRAP = new Pose2d(new Translation2d(), new Rotation2d());
    public static Pose2d BLUE_PICKUP = new Pose2d(new Translation2d(), new Rotation2d());

    public static double TRAP_TO_CHAIN_X = 0.0;
    public static double TRAP_TO_CHAIN_Y = 0.0;

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

      public static final NavigableMap<Double, Double> FLYWHEEL_DISTANCE_LUT =
          new TreeMap<>(Map.of(0.0, 0.0));

      public static final NavigableMap<Double, Double> ARM_DISTANCE_LUT =
          new TreeMap<>(Map.of(0.0, 0.0));
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

      public static final double ENCODER_RATIO = 1 * 360.0;
      public static final double MOTOR_RATIO = 1 * 360.0;

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
      public static final double VELOCITY = (10 / 60.0) * 360; // DEG/s
      public static final double ACCELERATION = (0.25 / 60.0) * 360.0; // DEG/s/s
      public static final double JERK = 10_000; // DEG/s/s/s

      public static final double POSITION_READY_TOLERANCE = 5; // DEG

      public static final double BASE_SHOT_POSITION = 65; // DEG
      public static final double AMP_POSITION = 50; // DEG
      public static final double TRAP_PREP_POSITION = 50; // DEG
      public static final double FULL_STOW_POSITION = 0; // DEG
      public static final double PARTIAL_STOW_POSITION = 50; // DEG
      public static final double CHUTE_INTAKE_POSITION = 60; // DEG

      public static final double AUTO_SYNC_TOLERANCE = 5;
      public static final double AUTO_SYNC_MAX_VELOCITY = 1; // DEG/s

      public static final boolean ENABLE_AUTO_SYNC = false;
      public static final double MIN_TIME_BETWEEN_SYNC = 2.0;

      public static final double VOLTAGE_INCREMENT = 0.25;

      public static final double MIN_ANGLE = 0.0;
      public static final double MAX_ANGLE = 0.0;
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
      public static final double BASE_SHOT_VELOCITY = 4000 / 60.0; // RPS
      public static final double SPIN_UP_READY_TOLERANCE = 60 / 60.0; // RPS

      public static final double PASS_THROUGH_SPEED = 5 / 60.0; // RPS

      public static final double CHUTE_INTAKE_SPEED = 600 / 60.0; // RPS
      public static final double AMP_SPEED = 1000 / 60.0; // RPS

      public static final double VOLTAGE_INCREMENT = 0.25;
    }
  }

  public static final class Intake {
    public static final class Hardware {
      public static final int TOP_ID = 0;
      public static final int BOTTOM_ID = 1;
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
      public static final int LEFT_CLIMBER_ID = 0;
      public static final int RIGHT_CLIMBER_ID = 0;

      public static final boolean LEFT_INVERTED = false;
      public static final boolean RIGHT_INVERTED = false;

      public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = DEFAULT_CURRENT_LIMIT;

      // rotations to meters
      public static final double CLIMBER_RATIO = 0;
    }

    public static final class Settings {
      public static final LoggedTunablePIDSV FREE_GAINS =
          new LoggedTunablePIDSV(
              "Climber Free Gains", new PIDSVGains(0, 0, 0, 0, 0), () -> ALLOW_TUNING);

      public static final LoggedTunablePIDSV LOADED_GAINS =
          new LoggedTunablePIDSV(
              "Climber Loaded Gains", new PIDSVGains(0, 0, 0, 0, 0), () -> ALLOW_TUNING);

      public static double FREE_VELOCITY = 0;
      public static double FREE_ACCELERATION = 0;
      public static double FREE_JERK = 0;

      public static double LOADED_VELOCITY = 0;
      public static double LOADED_ACCELERATION = 0;
      public static double LOADED_JERK = 0;

      // meters
      public static double SETPOINT_TOLERANCE = 0.01;

      public static int FREE_SLOT = 0;
      public static int LOADED_SLOT = 1;

      public static double EXTENSION_SETPOINT = 0; // meters

      public static double VOLTAGE_INCREMENT = 0.125;
    }
  }

  public static final class Indexer {
    public static final class Hardware {
      public static final int BELT_MOTOR_ID = 0;

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
      public static final double EXPECT_SPEED = 0.0;
      public static final double PASS_THROUGH_SPEED = 0.0;
      public static final double INDEX_SPEED = 0.0;
      public static final double FEED_SPEED = 0.0;

      public static final double INDEX_TIMEOUT = 0.0;

      public static final double VOLTAGE_INCREMENT = 0.0;
    }

    public static final class Sim {
      public static final double MOTOR_INERTIA = 0.0001;
    }
  }

  public static final class Drivetrain {
    public static final class Sim {}

    public static final class Hardware {
      public static final int GYRO_ID = 0;

      public static final String MODULE_CAN_BUS = "";
      public static final String GYRO_CAN_BUS = "";

      public static final ModuleInfo MODULE_1_INFO =
          ModuleInfo.generateModuleInfo(
              ModuleInfo.SwerveModuleType.MK4i,
              ModuleInfo.SwerveModuleSpeedLevel.L3,
              0, // DRIVE MOTOR ID
              0, // TURN MOTOR ID
              0, // ENCODER ID
              0.0, // ENCODER OFFSET
              new Translation2d(0, 0), // MODULE OFFSET FROM CENTER OF BOT
              false // DRIVE MOTOR INVERTED
              );

      public static final ModuleInfo MODULE_2_INFO =
          ModuleInfo.generateModuleInfo(
              ModuleInfo.SwerveModuleType.MK4i,
              ModuleInfo.SwerveModuleSpeedLevel.L3,
              0, // DRIVE MOTOR ID
              0, // TURN MOTOR ID
              0, // ENCODER ID
              0.0, // ENCODER OFFSET
              new Translation2d(0, 0), // MODULE OFFSET FROM CENTER OF BOT
              false // DRIVE MOTOR INVERTED
              );

      public static final ModuleInfo MODULE_3_INFO =
          ModuleInfo.generateModuleInfo(
              ModuleInfo.SwerveModuleType.MK4i,
              ModuleInfo.SwerveModuleSpeedLevel.L3,
              0, // DRIVE MOTOR ID
              0, // TURN MOTOR ID
              0, // ENCODER ID
              0.0, // ENCODER OFFSET
              new Translation2d(0, 0), // MODULE OFFSET FROM CENTER OF BOT
              false // DRIVE MOTOR INVERTED
              );

      public static final ModuleInfo MODULE_4_INFO =
          ModuleInfo.generateModuleInfo(
              ModuleInfo.SwerveModuleType.MK4i,
              ModuleInfo.SwerveModuleSpeedLevel.L3,
              0, // DRIVE MOTOR ID
              0, // TURN MOTOR ID
              0, // ENCODER ID
              0.0, // ENCODER OFFSET
              new Translation2d(0, 0), // MODULE OFFSET FROM CENTER OF BOT
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

      public static final double MAX_CHASSIS_SPEED = 0;
      public static final double MAX_CHASSIS_ACCELERATION = 0;
      public static final double MAX_CHASSIS_ROTATIONAL_SPEED = 0;
      public static final double MAX_CHASSIS_ROTATIONAL_ACCELERATION = 0;
      public static final double MAX_MODULE_TURN_SPEED = 0;
      public static final double MAX_MODULE_TURN_ACCELERATION = 0;

      public static final Matrix<N3, N1> STATE_STD_DEVIATIONS =
          VecBuilder.fill(0.003, 0.003, 0.0002);

      // meters and radians
      public static final SwerveSpeedLimits PATH_FIND_SPEED = new SwerveSpeedLimits(0, 0, 0, 0);
      public static final SwerveSpeedLimits TRAVERSE_SPEED = new SwerveSpeedLimits(0, 0, 0, 0);
      public static final SwerveSpeedLimits AMP_SPEED = new SwerveSpeedLimits(0, 0, 0, 0);
      public static final SwerveSpeedLimits INTAKE_SPEED = new SwerveSpeedLimits(0, 0, 0, 0);
      public static final SwerveSpeedLimits SPEAKER_SPEED = new SwerveSpeedLimits(0, 0, 0, 0);
      public static final SwerveSpeedLimits HUMAN_PLAYER_PICKUP_SPEED =
          new SwerveSpeedLimits(0, 0, 0, 0);
      public static final SwerveSpeedLimits TRAP_SPEED = new SwerveSpeedLimits(0, 0, 0, 0);

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

  public static double lerp(double a, double b, double t) {
    // found this on wiki
    return (1 - t) * a + t * b;
  }

  public static double[] getTrapOffsetFromBot(double climberExtension, double botAngle) {
    double trapToChainX = PhysicalConstants.TRAP_TO_CHAIN_X;
    double trapToChainY = PhysicalConstants.TRAP_TO_CHAIN_Y;
    double chainToBotX = PhysicalConstants.CLIMBER_X_DISTANCE_FROM_SHOOTER_PIVOT;
    double chainToBotY = PhysicalConstants.CLIMBER_Y_DISTANCE_FROM_SHOOTER_PIVOT + climberExtension;

    // https://www.desmos.com/calculator/tx6sop2qvm
    double x = chainToBotX * Math.cos(botAngle) - chainToBotY * Math.sin(botAngle) + trapToChainX;
    double y = chainToBotY * Math.cos(botAngle) + chainToBotX * Math.sin(botAngle) + trapToChainY;

    return new double[] {x, y};
  }

  public static Pose2d mirror(Pose2d pose) {
    return new Pose2d(
        new Translation2d(
            PhysicalConstants.APRIL_TAG_FIELD_LAYOUT.getFieldLength() - pose.getX(),
            PhysicalConstants.APRIL_TAG_FIELD_LAYOUT.getFieldWidth() - pose.getY()),
        pose.getRotation().rotateBy(new Rotation2d(Math.PI)));
  }

  public static Rotation2d rotationBetween(Pose2d pose1, Pose2d pose2) {
    return Rotation2d.fromRadians(
        Math.atan2(pose2.getY() - pose1.getY(), pose2.getX() - pose1.getX()));
  }
}
