package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.ShamLib.ShamLibConstants;
import frc.robot.ShamLib.motors.talonfx.PIDSVGains;

public class Constants {
  public static ShamLibConstants.BuildMode currentBuildMode = ShamLibConstants.BuildMode.SIM;

  public static final CurrentLimitsConfigs DEFAULT_CURRENT_LIMIT =
      new CurrentLimitsConfigs().withSupplyCurrentLimit(20).withSupplyCurrentLimitEnable(true);

  public static final class PhysicalConstants {
    // METERS
    public static final Translation3d CHASSIS_TO_INTAKE = new Translation3d(0, 0, 0);

    public static final Translation3d CHASSIS_TO_SHOOTER = new Translation3d(0, 0, 0);

    public static Translation3d SHOOTER_TO_ELEVATOR = new Translation3d(0, 0, 0);

    public static Translation3d ELEVATOR_TO_CLAW = new Translation3d(0, 0, 0);
  }

  // TODO: UPDATE INTAKE CONSTANTS
  public static final class Intake {
    public static final class Hardware {
      public static final int TOP_ID = 1;
      public static final int BOTTOM_ID = 0;

      public static final double TOP_RATIO = 1;
      public static final double BOTTOM_RATIO = 1;

      public static final boolean TOP_INVERTED = false;
      public static final boolean BOTTOM_INVERTED = false;

      public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;

      public static final CurrentLimitsConfigs CURRENT_LIMIT = DEFAULT_CURRENT_LIMIT;

      public static final PIDSVGains BOTTOM_GAINS = new PIDSVGains(0, 0, 0, 0, 0);

      public static final PIDSVGains TOP_GAINS = new PIDSVGains(0, 0, 0, 0, 0);
    }

    public static final class Sim {
      public static final double BELT_INERTIA = 0.001;
    }

    public static final class Settings {
      public static final double BELT_SPEED = 1;

      public static final double VOLTAGE_INC = 0.125;
    }
  }

  public static DriverStation.Alliance alliance = DriverStation.Alliance.Red;
  public static boolean overrideAlliance = false;

  public static void pullAllianceFromFMS() {
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
