package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import frc.robot.ShamLib.motors.talonfx.MotionMagicTalonFX;
import frc.robot.ShamLib.motors.talonfx.PIDSVGains;
import frc.robot.ShamLib.motors.talonfx.VelocityTalonFX;

public class IntakeIOReal implements IntakeIO {
    private final PIDSVGains armGains;
    private final PIDSVGains beltGains;

    private final MotionMagicTalonFX armMotor;
    private final VelocityTalonFX beltMotor;

    public IntakeIOReal(
            int armID,
            PIDSVGains armGains,
            double armRatio,
            double armMaxVelo,
            double armMaxAccel,
            double armMaxJerk,
            int beltID,
            PIDSVGains beltGains,
            double beltRatio,
            CurrentLimitsConfigs currentLimitsConfigs
    ) {
        this.armGains = armGains;
        this.beltGains = beltGains;

        armMotor = new MotionMagicTalonFX(armID, armGains, armRatio, armMaxVelo, armMaxAccel, armMaxJerk);
        beltMotor = new VelocityTalonFX(beltID, beltGains, beltRatio);


    }

    @Override
    public void setBeltTargetVelocity(double velocity) {

    }

    @Override
    public void setArmTargetVelocity(double velocity) {

    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {

    }

    private void applyCurrentConfig(CurrentLimitsConfigs current, )
}
