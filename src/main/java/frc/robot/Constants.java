package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.ShamLib.ShamLibConstants;
import frc.robot.ShamLib.motors.talonfx.PIDSVGains;
import frc.robot.ShamLib.motors.tuning.LoggedTunablePIDSV;
import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;

public class Constants {
  public static ShamLibConstants.BuildMode currentBuildMode = ShamLibConstants.BuildMode.SIM;
  public static final CurrentLimitsConfigs DEFAULT_CURRENT_LIMIT =
      new CurrentLimitsConfigs().withSupplyCurrentLimit(20).withSupplyCurrentLimitEnable(true);

  public static final boolean ALLOW_TUNING = true;

  public static final class PhysicalConstants {
    // METERS

    public static final Pose3d CHASSIS_TO_SHOOTER =
        new Pose3d(new Translation3d(), new Rotation3d());

    public static Pose3d SPEAKER_POSE = new Pose3d(new Translation3d(0, 0, 0), new Rotation3d());

    // how much taller climber is than shooter pivot when stowed
    public static double CLIMBER_Y_DISTANCE_FROM_SHOOTER_PIVOT = 0.0;

    // how far away climber is from shooter pivot on front/back axis
    public static double CLIMBER_X_DISTANCE_FROM_SHOOTER_PIVOT = 0.0;

    public static double TRAP_TO_CHAIN_X = 0.0;
    public static double TRAP_TO_CHAIN_Y = 0.0;

    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT;

    static {
      try {
        APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      }
      catch (Exception e) {
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

      public static final double VOLTAGE_INCREMENT = 0.25;
    }
  }

  // TODO: UPDATE INTAKE CONSTANTS
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

  public static DriverStation.Alliance alliance = DriverStation.Alliance.Red;
  public static boolean overrideAlliance = false;

  public static final class Indexer {
    public static final class Hardware {
      public static final int BELT_MOTOR_ID = 0;

      public static final double BELT_RATIO = 1.0;

      public static final boolean INVERT_BELT_MOTOR = false;

      public static final int PROX_1_ID = 0;
      public static final int PROX_2_ID = 1;
      public static final int PROX_3_ID = 2;

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

  public static void pullAllianceFromFMS(RobotContainer rc) {
    boolean isRedAlliance =
        NetworkTableInstance.getDefault()
            .getTable("FMSInfo")
            .getEntry("IsRedAlliance")
            .getBoolean(true);
    if (!overrideAlliance) {
      alliance = isRedAlliance ? DriverStation.Alliance.Red : DriverStation.Alliance.Blue;
    }
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
}
