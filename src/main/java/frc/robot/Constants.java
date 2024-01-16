package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.ShamLib.ShamLibConstants;
import frc.robot.ShamLib.motors.talonfx.PIDSVGains;

public class Constants {
  public static ShamLibConstants.BuildMode currentBuildMode = ShamLibConstants.BuildMode.REPLAY;

  public static final CurrentLimitsConfigs DEFAULT_CURRENT_LIMIT =
      new CurrentLimitsConfigs().withSupplyCurrentLimit(20).withSupplyCurrentLimitEnable(true);

  public static final class PhysicalConstants {
    // METERS
    public static final Translation3d CHASSIS_TO_INTAKE = new Translation3d(0, 0, 0);

    public static final Translation3d CHASSIS_TO_SHOOTER = new Translation3d(0, 0, 0);

    public static Translation3d SHOOTER_TO_ELEVATOR = new Translation3d(0, 0, 0);

    public static Translation3d ELEVATOR_TO_CLAW = new Translation3d(0, 0, 0);
  }

  public static final class Flywheel {
    public static final class Sim {}

    public static final class Hardware {
      public static int TOP_MOTOR_ID = 0;
      public static int BOTTOM_MOTOR_ID = 0;

      public static final double TOP_MOTOR_RATIO = 0.0;
      public static final double BOTTOM_MOTOR_RATIO = 0.0;

      public static final boolean TOP_MOTOR_INVERTED = false;
      public static final boolean BOTTOM_MOTOR_INVERTED = false;

      public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;

      public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = DEFAULT_CURRENT_LIMIT;

      public static final PIDSVGains TOP_MOTOR_GAINS = new PIDSVGains(0, 0, 0, 0, 0);

      public static final PIDSVGains BOTTOM_MOTOR_GAINS = new PIDSVGains(0, 0, 0, 0, 0);
    }

    public static final class Settings {
      public static final double BASE_SHOT_VELOCITY = 0.0; //RPS
      public static final double SPIN_UP_READY_TOLERANCE = 1; //RPS

      public static final double PASS_THROUGH_SPEED = 0.0; //RPS

      public static final double CHUTE_INTAKE_SPEED = 0.0; //RPS
    }
  }

  public static DriverStation.Alliance alliance = DriverStation.Alliance.Red;
  public static boolean overrideAlliance = false;

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
}
