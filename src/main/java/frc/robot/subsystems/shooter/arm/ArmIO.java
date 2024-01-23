package frc.robot.subsystems.shooter.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public class ArmInputs {
    public double motorPosition = 0.0; // DEG
    public double targetPosition = 0.0; // DEG
    public double encoderPosition = 0.0; // DEG

    public double motorVelocity = 0.0; // DEG/s
    public double motorVoltage = 0.0;
  }

  public default void updateInputs(ArmInputs inputs) {}

  public default void setTargetPosition(double position) {}

  public default void stop() {}

  public default void syncToAbsoluteEncoder() {}

  public default void setVoltage(double voltage) {}
}
