package frc.robot.subsystems.lights;


import frc.robot.ShamLib.Candle.CANdleEX;
import frc.robot.ShamLib.Candle.RGB;

import static frc.robot.Constants.Lights.Hardware.*;

import com.ctre.phoenix.led.Animation;

public class LightsIOReal implements LightsIO {

    private final CANdleEX candle1 = new CANdleEX(CANDLE_ID, BRIGHTNESS, NUM_LIGHTS);

    public LightsIOReal() {

    }

    

    @Override
    public void updateInputs(LightsInputs inputs) {
        inputs.rail5VVoltage1 = candle1.get5VRailVoltage();
        inputs.busVoltage1 = candle1.getBusVoltage();

    }



    @Override
    public void animate(Animation animation) {
        candle1.animate(animation);
    }

    @Override
    public void setLEDs(RGB values) {
        candle1.setLEDs(values);
    }

    

}
