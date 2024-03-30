package frc.robot.subsystems.shooter.flywheel;

import frc.robot.ShamLib.motors.talonfx.PIDSVGains;
import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public class FlywheelInputs {
    public double topVelocity = 0.0;
    public double topRotorVelocity = 0.0;
    public double topTargetVelocity = 0.0;
    public double topVoltage = 0.0;


    public double bottomVelocity = 0.0;
    public double bottomRotorVelocity = 0.0;
    public double bottomTargetVelocity = 0.0;
    public double bottomVoltage = 0.0;

  }

  public default void setFlywheelTarget(double target) {}

  public default void setFlywheelTargets(double targetTop, double targetBottom) {}

  public default void stop() {}

  public default void setTopVoltage(double voltage) {}
  public default void setBottomVoltage(double voltage) {}

  public default void setTopGains(PIDSVGains gains) {}

  public default void setBottomGains(PIDSVGains gains) {}

  public default void resetFollower() {}

  public default void updateInputs(FlywheelInputs inputs) {}
}
