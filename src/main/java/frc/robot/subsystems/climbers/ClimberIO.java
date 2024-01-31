package frc.robot.subsystems.climbers;

import frc.robot.ShamLib.motors.talonfx.PIDSVGains;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public class ClimberInputs {
    public double position = 0;
    public double targetPosition = 0;

    public double voltage = 0;
  }

  public default void updateInputs(ClimberInputs inputs) {}

  public default void setControlSlot(int slot) {}

  public default void setSlot0Gains(PIDSVGains gains) {}

  public default void setSlot1Gains(PIDSVGains gains) {}

  public default void setSpeed(double velocity, double acceleration, double jerk) {}

  public default void stop() {}

  public default void setTarget(double target) {}

  public default void setVoltage(double voltage) {}

  public default void resetPosition() {
    resetPosition(0);
  }

  public default void resetPosition(double position) {}
}
