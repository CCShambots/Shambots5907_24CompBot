package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public class FlywheelInputs {
    public double topVelocity = 0.0; // RPS
    public double topTargetVelocity = 0.0; // RPS

    public double bottomVelocity = 0.0; // RPS
    public double bottomTargetVelocity = 0.0; // RPS
  }

  public default void setFlywheelTarget(double target) {}

  public default void stop() {}

  public default void setTopVoltage(double voltage) {}

  public default void setBottomVoltage(double voltage) {}

  public default void setVoltage(double voltage) {
    setTopVoltage(voltage);
    setBottomVoltage(voltage);
  }

  public default void updateInputs(FlywheelInputs inputs) {}
}
