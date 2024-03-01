package frc.robot.subsystems.lights;

import static frc.robot.Constants.Lights.Hardware.*;

import com.ctre.phoenix.led.Animation;
import frc.robot.ShamLib.Candle.CANdleEX;
import frc.robot.ShamLib.Candle.MultipleColorSegments;
import frc.robot.ShamLib.Candle.RGB;

public class LightsIOReal implements LightsIO {

  private final CANdleEX candle = new CANdleEX(CANDLE_ID, BRIGHTNESS, NUM_LIGHTS);

  public LightsIOReal() {}

  @Override
  public void updateInputs(LightsInputs inputs) {
    inputs.rail5VVoltage1 = candle.get5VRailVoltage();
    inputs.busVoltage1 = candle.getBusVoltage();
  }

  @Override
  public void animate(Animation animation) {
    candle.animate(animation);
  }

  @Override
  public void setLEDs(RGB values) {
    candle.setLEDs(values);
  }

  @Override
  public void setMultipleSegs(MultipleColorSegments segs) {
    candle.setLEDs(segs);
  }
}
