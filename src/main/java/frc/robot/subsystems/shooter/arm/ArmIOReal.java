package frc.robot.subsystems.shooter.arm;

import static frc.robot.Constants.Arm.Hardware.*;
import static frc.robot.Constants.Arm.Settings.*;

import com.ctre.phoenix6.controls.StrictFollower;
import frc.robot.ShamLib.motors.talonfx.MotionMagicTalonFX;
import frc.robot.ShamLib.sensor.ThroughBoreEncoder;

public class ArmIOReal implements ArmIO {
    protected final MotionMagicTalonFX leaderMotor = new MotionMagicTalonFX(
            LEADER_ID,
            GAINS,
            MOTOR_RATIO,
            VELOCITY,
            ACCELERATION,
            JERK
    );

    protected final MotionMagicTalonFX followerMotor = new MotionMagicTalonFX(
            FOLLOWER_ID,
            GAINS,
            MOTOR_RATIO,
            VELOCITY,
            ACCELERATION,
            JERK
    );

    private final ThroughBoreEncoder encoder = new ThroughBoreEncoder(
            ENCODER_ID,
            ENCODER_OFFSET
    );

    public ArmIOReal() {
        this(false);
    }

    public ArmIOReal(boolean sim) {
        if (!sim) configureCurrentLimits();

        configureHardware();
        syncToAbsoluteEncoder();
    }

    @Override
    public void updateInputs(ArmInputs inputs) {
        inputs.encoderPosition = getEncoderPosition();
        inputs.motorPosition = leaderMotor.getEncoderPosition();
        inputs.targetPosition = leaderMotor.getTarget();
        inputs.motorVelocity = leaderMotor.getEncoderVelocity();
    }

    @Override
    public void setTargetPosition(double position) {
        leaderMotor.setTarget(position);
        followerMotor.setControl(new StrictFollower(LEADER_ID));
    }

    @Override
    public void stop() {
        leaderMotor.stopMotor();
        followerMotor.stopMotor();
    }

    @Override
    public void syncToAbsoluteEncoder() {
        leaderMotor.resetPosition(getEncoderPosition());
    }

    @Override
    public void setVoltage(double voltage) {
        leaderMotor.setVoltage(voltage);
        followerMotor.setControl(new StrictFollower(LEADER_ID));
    }

    private double getEncoderPosition() {
        return encoder.getRaw() * ENCODER_RATIO;
    }

    private void configureCurrentLimits() {
        leaderMotor.getConfigurator().apply(CURRENT_LIMITS_CONFIGS);
        followerMotor.getConfigurator().apply(CURRENT_LIMITS_CONFIGS);
    }

    private void configureHardware() {
        leaderMotor.setNeutralMode(NEUTRAL_MODE);
        followerMotor.setNeutralMode(NEUTRAL_MODE);

        leaderMotor.setInverted(LEADER_INVERTED);
        followerMotor.setInverted(FOLLOWER_INVERTED);

        encoder.setInverted(ENCODER_INVERTED);
    }
}
