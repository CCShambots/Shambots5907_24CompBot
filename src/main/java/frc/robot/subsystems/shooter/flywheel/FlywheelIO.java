package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public static class FlywheelInputs {
    public double velocity = 0.0; //RPS
    public double targetVelocity = 0.0; //RPS
  }

  public default void setFlywheelTarget(double target) {}

  public default void setFlywheelVoltage(double voltage) {}

  public default void updateInputs(FlywheelInputs inputs) {}
}
