package frc.robot.subsystems.shooter.arm;

import frc.robot.ShamLib.motors.talonfx.PIDSVGains;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public class ArmInputs {
    public double motorPosition = 0.0;
    public double targetPosition = 0.0;
    public double encoderPosition = 0.0;

    public double motorRotorVelocity = 0.0;
    public double motorVelocity = 0.0;
    public double motorVoltage = 0.0;
  }

  public default void updateInputs(ArmInputs inputs) {}

  public default void setTargetPosition(double position) {}

  public default void stop() {}

  public default void syncToAbsoluteEncoder() {}

  public default void resetFollower() {}

  public default void setGains(PIDSVGains gains) {}

  public default void setVoltage(double voltage) {}
}
