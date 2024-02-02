package frc.robot.fmsinfo;

import org.littletonrobotics.junction.AutoLog;

public interface FMSIO {

  @AutoLog
  public class FMSInputs {
    public boolean isRedAlliance = true;
    public String eventName = "";
    public double matchTimeRemaining = 0;
    public boolean connected = false;
  }

  public default void updateInputs(FMSInputs inputs) {}
}
