package frc.robot.subsystems.lights;

import com.ctre.phoenix.led.Animation;
import frc.robot.ShamLib.Candle.MultipleColorSegments;
import frc.robot.ShamLib.Candle.RGB;
import org.littletonrobotics.junction.AutoLog;

public interface LightsIO {

  @AutoLog
  public class LightsInputs {
    public double rail5VVoltage1 = 0;
    public double busVoltage1 = 0;
  }

  public default void updateInputs(LightsInputs inputs) {}

  public default void setLEDs(RGB values) {}

  public default void animate(Animation animation) {}

  public default void setMultipleSegs(MultipleColorSegments segs) {}
}
