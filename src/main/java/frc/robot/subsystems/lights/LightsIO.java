package frc.robot.subsystems.lights;

import org.littletonrobotics.junction.AutoLog;

public interface LightsIO {
    
    @AutoLog
    public class LightsInputs {

    }

    public default void updateInputs(LightsInputs inputs) {}
}
