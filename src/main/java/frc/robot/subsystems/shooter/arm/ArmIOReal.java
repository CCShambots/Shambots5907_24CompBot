package frc.robot.subsystems.shooter.arm;

import static frc.robot.Constants.Arm.Hardware.*;
import static frc.robot.Constants.Arm.Settings.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import frc.robot.ShamLib.motors.talonfx.MotionMagicTalonFX;
import frc.robot.ShamLib.motors.talonfx.PIDSVGains;

public class ArmIOReal implements ArmIO {
  protected final MotionMagicTalonFX leaderMotor =
      new MotionMagicTalonFX(LEADER_ID, GAINS.get(), MOTOR_RATIO, VELOCITY, ACCELERATION, JERK);

  protected final MotionMagicTalonFX followerMotor =
      new MotionMagicTalonFX(FOLLOWER_ID, GAINS.get(), MOTOR_RATIO, VELOCITY, ACCELERATION, JERK);

  private final AnalogPotentiometer potentiometer =
      new AnalogPotentiometer(
          ENCODER_ID, ENCODER_RATIO * (ENCODER_INVERTED ? -1 : 1), ENCODER_OFFSET);

  public ArmIOReal() {
    this(false);
  }

  public ArmIOReal(boolean sim) {
    if (!sim) configureCurrentLimits();

    configureHardware();
    syncToAbsoluteEncoder();

    GAINS.setOnChange(this::setGains);
  }

  @Override
  public void updateInputs(ArmInputs inputs) {
    inputs.encoderPosition = potentiometer.get();
    inputs.motorPosition = leaderMotor.getEncoderPosition();
    inputs.targetPosition = leaderMotor.getTarget();
    inputs.motorVelocity = leaderMotor.getEncoderVelocity();
    inputs.motorVoltage = leaderMotor.getMotorVoltage().getValueAsDouble();
    inputs.motorRotorVelocity = leaderMotor.getRotorVelocity().getValueAsDouble();
  }

  @Override
  public void resetFollower() {
    followerMotor.setControl(new Follower(LEADER_ID, FOLLOWER_INVERTED));
  }

  @Override
  public void setTargetPosition(double position) {
    if (position < MAX_ANGLE && position > MIN_ANGLE) {
      leaderMotor.setTarget(position);
    }
  }

  @Override
  public void stop() {
    leaderMotor.stopMotor();
    followerMotor.stopMotor();
  }

  @Override
  public void syncToAbsoluteEncoder() {
    leaderMotor.resetPosition(potentiometer.get());
  }

  @Override
  public void setVoltage(double voltage) {
    leaderMotor.setVoltage(voltage);
  }

  @Override
  public void setGains(PIDSVGains gains) {
    leaderMotor
        .getConfigurator()
        .apply(
            new Slot0Configs()
                .withKP(gains.getP())
                .withKI(gains.getI())
                .withKD(gains.getD())
                .withKS(gains.getS())
                .withKV(gains.getV()));

    followerMotor
        .getConfigurator()
        .apply(
            new Slot0Configs()
                .withKP(gains.getP())
                .withKI(gains.getI())
                .withKD(gains.getD())
                .withKS(gains.getS())
                .withKV(gains.getV()));
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
  }
}
