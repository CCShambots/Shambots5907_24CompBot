package frc.robot.subsystems.shooter.flywheel;

import frc.robot.ShamLib.motors.talonfx.PIDSVGains;
import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public class FlywheelInputs {
    public double velocity = 0.0;
    public double rotorVelocity = 0.0;
    public double targetVelocity = 0.0;
    public double voltage = 0.0;
  }

  public default void setFlywheelTarget(double target) {}

  public default void stop() {}

  public default void setVoltage(double voltage) {}

  public default void setGains(PIDSVGains gains) {}

  public default void resetFollower() {}

  public default void updateInputs(FlywheelInputs inputs) {}
}
