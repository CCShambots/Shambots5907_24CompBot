package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.ShamLib.ShamLibConstants;
import frc.robot.ShamLib.motors.talonfx.PIDSVGains;

public class Constants {
  public static ShamLibConstants.BuildMode currentBuildMode = ShamLibConstants.BuildMode.REPLAY;

  //TODO: UPDATE INTAKE CONSTANTS
  public static final class Intake {
    public static final int ARM_ID = 0;
    public static final int BELT_ID = 0;

    public static final double ARM_RATIO = 0.0;
    public static final double ARM_MAX_VELOCITY = 0.0;
    public static final double ARM_MAX_ACCELERATION = 0.0;
    public static final double ARM_MAX_JERK = 0.0;

    public static final double BELT_RATIO = 0.0;

    public static final PIDSVGains ARM_GAINS = new PIDSVGains(
            0,
            0,
            0,
            0,
            0
    );

    public static final PIDSVGains BELT_GAINS = new PIDSVGains(
            0,
            0,
            0,
            0,
            0
    );
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
}
