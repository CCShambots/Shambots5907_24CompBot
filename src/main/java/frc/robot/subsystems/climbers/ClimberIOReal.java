package frc.robot.subsystems.climbers;

import static frc.robot.Constants.Climbers.Hardware.*;
import static frc.robot.Constants.Climbers.Settings.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import frc.robot.ShamLib.motors.talonfx.MotionMagicTalonFX;
import frc.robot.ShamLib.motors.talonfx.PIDSVGains;

public class ClimberIOReal implements ClimberIO {
  protected MotionMagicTalonFX motor;
  private MotionMagicVoltage mmReq = new MotionMagicVoltage(0);

  private double target = 0;

  public ClimberIOReal(int motorID, boolean inverted) {
    this(motorID, inverted, false);
  }

  public ClimberIOReal(int motorID, boolean inverted, boolean sim) {
    motor =
        new MotionMagicTalonFX(
            motorID, FREE_GAINS.get(), CLIMBER_RATIO, FREE_VELOCITY, FREE_ACCELERATION, FREE_JERK);

    if (!sim) motor.getConfigurator().apply(CURRENT_LIMITS_CONFIGS);

    configureHardware();

    motor.setInverted(inverted);

    FREE_GAINS.setOnChange(this::setSlot0Gains);
    LOADED_GAINS.setOnChange(this::setSlot1Gains);
  }

  @Override
  public void updateInputs(ClimberInputs inputs) {
    inputs.targetPosition = target;
    inputs.position = motor.getEncoderPosition();

    inputs.voltage = motor.getMotorVoltage().getValue();
    inputs.velocity = motor.getEncoderVelocity();

    inputs.rotorVelocity = motor.getRotorVelocity().getValue();
  }

  @Override
  public void setSpeed(double velocity, double acceleration, double jerk) {
    motor.changeSpeed(velocity, acceleration, jerk);
  }

  @Override
  public void setControlSlot(int slot) {
    mmReq = mmReq.withSlot(slot);
  }

  @Override
  public void setSlot0Gains(PIDSVGains gains) {
    Slot0Configs configs = new Slot0Configs();

    motor.getConfigurator().refresh(configs);

    configs =
        configs
            .withKP(gains.getP())
            .withKI(gains.getI())
            .withKD(gains.getD())
            .withKV(gains.getV())
            .withKS(gains.getS());

    motor.getConfigurator().apply(configs);
  }

  @Override
  public void setSlot1Gains(PIDSVGains gains) {
    Slot1Configs configs = new Slot1Configs();

    motor.getConfigurator().refresh(configs);

    configs =
        configs
            .withKP(gains.getP())
            .withKI(gains.getI())
            .withKD(gains.getD())
            .withKV(gains.getV())
            .withKS(gains.getS());

    motor.getConfigurator().apply(configs);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void setTarget(double target) {
    this.target = target;
    motor.setControl(mmReq.withPosition(motor.outputToTicks(target)).withSlot(0));
  }

  @Override
  public void setPower(double power) {
    motor.setManualPower(power);
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public void resetPosition(double position) {
    motor.resetPosition(position);
  }

  private void configureHardware() {
    motor.setNeutralMode(NEUTRAL_MODE);
  }
}
