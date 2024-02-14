package frc.robot.subsystems.shooter.flywheel;

import static frc.robot.Constants.Flywheel.Hardware.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import frc.robot.ShamLib.motors.talonfx.PIDSVGains;
import frc.robot.ShamLib.motors.talonfx.VelocityTalonFX;

public class FlywheelIOReal implements FlywheelIO {
  protected final VelocityTalonFX topMotor =
      new VelocityTalonFX(TOP_MOTOR_ID, GAINS.get(), TOP_MOTOR_RATIO);
  protected final VelocityTalonFX bottomMotor =
      new VelocityTalonFX(BOTTOM_MOTOR_ID, GAINS.get(), BOTTOM_MOTOR_RATIO);

  public FlywheelIOReal() {
    this(false);
  }

  public FlywheelIOReal(boolean sim) {
    if (!sim) configureCurrentLimits();
    configureHardware();

    GAINS.setOnChange(this::setGains);
  }

  @Override
  public void updateInputs(FlywheelInputs inputs) {
    inputs.targetVelocity = topMotor.getTarget();
    inputs.velocity = topMotor.getEncoderVelocity();
    inputs.voltage = topMotor.getMotorVoltage().getValueAsDouble();
    inputs.rotorVelocity = topMotor.getRotorVelocity().getValueAsDouble();
  }

  @Override
  public void setFlywheelTarget(double target) {
    topMotor.setTarget(target);
    bottomMotor.setTarget(target);
  }

  @Override
  public void setVoltage(double voltage) {
    topMotor.setVoltage(voltage);
  }

  @Override
  public void stop() {
    topMotor.stopMotor();
    bottomMotor.stopMotor();
  }

  @Override
  public void resetFollower() {
    bottomMotor.setControl(new Follower(TOP_MOTOR_ID, BOTTOM_MOTOR_INVERTED));
  }

  @Override
  public void setGains(PIDSVGains gains) {
    topMotor
        .getConfigurator()
        .apply(
            new Slot0Configs()
                .withKP(gains.getP())
                .withKI(gains.getI())
                .withKD(gains.getD())
                .withKS(gains.getS())
                .withKV(gains.getV()));
  }

  private void configureHardware() {
    topMotor.setNeutralMode(NEUTRAL_MODE);
    bottomMotor.setNeutralMode(NEUTRAL_MODE);

    topMotor.setInverted(TOP_MOTOR_INVERTED);
  }

  private void configureCurrentLimits() {
    topMotor.getConfigurator().apply(CURRENT_LIMITS_CONFIGS);
    bottomMotor.getConfigurator().apply(CURRENT_LIMITS_CONFIGS);
  }
}
