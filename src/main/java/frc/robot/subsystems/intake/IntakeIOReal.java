package frc.robot.subsystems.intake;

import static frc.robot.Constants.Intake.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import frc.robot.ShamLib.motors.talonfx.MotionMagicTalonFX;
import frc.robot.ShamLib.motors.talonfx.PIDSVGains;
import frc.robot.ShamLib.motors.talonfx.VelocityTalonFX;

public class IntakeIOReal implements IntakeIO {
    private final MotionMagicTalonFX armMotor;
    private final VelocityTalonFX beltMotor;

    public IntakeIOReal(
            CurrentLimitsConfigs currentLimitsConfigs
    ) {
        armMotor = new MotionMagicTalonFX(
                ARM_ID,
                ARM_GAINS,
                ARM_RATIO,
                ARM_MAX_VELOCITY,
                ARM_MAX_ACCELERATION,
                ARM_MAX_JERK
        );

        beltMotor = new VelocityTalonFX(
                BELT_ID,
                BELT_GAINS,
                BELT_RATIO
        );
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
