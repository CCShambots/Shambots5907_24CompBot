package frc.robot.subsystems.intake;

import frc.robot.ShamLib.motors.talonfx.PIDSVGains;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double velocity = 0.0; // RPM
    public double targetVelocity = 0.0; // RPM
    public double voltage = 0.0;
  }

  public default void resetFollower() {}

  public default void setBeltTargetVelocity(double velocity) {}

  public default void stop() {}

  public default void setVoltage(double voltage) {}

  public default void setGains(PIDSVGains gains) {}

  public default void updateInputs(IntakeIOInputs inputs) {}
}
