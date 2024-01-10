package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.ShamLib.ShamLibConstants;
import frc.robot.ShamLib.motors.talonfx.PIDSVGains;

public class Constants {
  public static ShamLibConstants.BuildMode currentBuildMode = ShamLibConstants.BuildMode.REPLAY;

  public static final CurrentLimitsConfigs DEFAULT_CURRENT_LIMIT =
      new CurrentLimitsConfigs().withSupplyCurrentLimit(20).withSupplyCurrentLimitEnable(true);

  // TODO: UPDATE INTAKE CONSTANTS
  public static final class Intake {
    public static final class Hardware {
      public static final int ARM_ENCODER_ID = 0;
      public static final int ARM_ID = 0;
      public static final int BELT_ID = 0;

      public static final double ARM_ENCODER_RATIO = 1.0;
      public static final double ARM_RATIO = 1.0;
      public static final double BELT_RATIO = 1.0;

      public static final boolean ARM_ENCODER_INVERTED = false;
      public static final boolean BELT_INVERTED = false;
      public static final boolean ARM_INVERTED = false;

      public static final NeutralModeValue ARM_NEUTRAL_MODE = NeutralModeValue.Brake;
      public static final NeutralModeValue BELT_NEUTRAL_MODE = NeutralModeValue.Coast;

      public static final CurrentLimitsConfigs ARM_CURRENT_LIMIT = DEFAULT_CURRENT_LIMIT;
      public static final CurrentLimitsConfigs BELT_CURRENT_LIMIT = DEFAULT_CURRENT_LIMIT;

      public static final PIDSVGains ARM_GAINS = new PIDSVGains(0, 0, 0, 0, 0);

      public static final PIDSVGains BELT_GAINS = new PIDSVGains(0, 0, 0, 0, 0);
    }

    public static final class Settings {
      public static final double STOW_ANGLE = 0.0; // DEG
      public static final double DEPLOY_ANGLE = 0.0; // DEG

      public static final double BELT_SPEED = 0.0; // DEG/s

      public static final double ARM_VELOCITY = 0.0; // DEG/s
      public static final double ARM_ACCELERATION = 0.0; // DEG/s/s
      public static final double ARM_JERK = 0.0; // DEG/s/s/s (lol)

      public static final double ARM_ENCODER_OFFSET = 0.0; // DEG

      public static final double STOW_EXPEL_DURATION = 0.0; //SECONDS

      public static final double ANGLE_SETPOINT_TOLERANCE = 0.0; //DEGREES
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
    return doubleEqual(a, b, 0.00001); //TODO: idk if this is fine or not
  }
}
