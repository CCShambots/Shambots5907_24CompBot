package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double topVelocity = 0.0; // RPM
    public double topTargetVelocity = 0.0; // RPM
    public double topVoltage = 0.0;

    public double bottomVelocity = 0.0;
    public double bottomTargetVelocity = 0.0;
    public double bottomVoltage = 0.0;
  }

  public default void setBeltTargetVelocity(double velocity) {}

  public default void stop() {}

  public default void setTopVoltage() {}

  public default void setBottomVoltage() {}

  public default void updateInputs(IntakeIOInputs inputs) {}
}
