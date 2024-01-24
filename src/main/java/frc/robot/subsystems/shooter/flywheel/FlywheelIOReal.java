package frc.robot.subsystems.shooter.flywheel;

import static frc.robot.Constants.Flywheel.Hardware.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import frc.robot.ShamLib.motors.talonfx.PIDSVGains;
import frc.robot.ShamLib.motors.talonfx.VelocityTalonFX;

public class FlywheelIOReal implements FlywheelIO {
  protected final VelocityTalonFX topMotor =
      new VelocityTalonFX(TOP_MOTOR_ID, TOP_MOTOR_GAINS.get(), TOP_MOTOR_RATIO);
  protected final VelocityTalonFX bottomMotor =
      new VelocityTalonFX(BOTTOM_MOTOR_ID, BOTTOM_MOTOR_GAINS.get(), BOTTOM_MOTOR_RATIO);

  public FlywheelIOReal() {
    this(false);
  }

  public FlywheelIOReal(boolean sim) {
    if (!sim) configureCurrentLimits();
    configureHardware();

    TOP_MOTOR_GAINS.setOnChange(this::setTopGains);
    BOTTOM_MOTOR_GAINS.setOnChange(this::setBottomGains);
  }

  @Override
  public void updateInputs(FlywheelInputs inputs) {
    inputs.topTargetVelocity = topMotor.getTarget();
    inputs.topVelocity = topMotor.getEncoderVelocity();

    inputs.bottomTargetVelocity = bottomMotor.getTarget();
    inputs.bottomVelocity = bottomMotor.getEncoderVelocity();

    inputs.topVoltage = topMotor.getMotorVoltage().getValueAsDouble();
    inputs.bottomVelocity = bottomMotor.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void setFlywheelTarget(double target) {
    topMotor.setTarget(target);
    bottomMotor.setTarget(target);
  }

  @Override
  public void setBottomVoltage(double voltage) {
    bottomMotor.setVoltage(voltage);
  }

  @Override
  public void setTopVoltage(double voltage) {
    topMotor.setVoltage(voltage);
  }

  @Override
  public void stop() {
    topMotor.stopMotor();
    bottomMotor.stopMotor();
  }

  @Override
  public void setBottomGains(PIDSVGains gains) {
    bottomMotor.getConfigurator().apply(
            new Slot0Configs()
                    .withKP(gains.getP())
                    .withKI(gains.getI())
                    .withKD(gains.getD())
                    .withKS(gains.getS())
                    .withKV(gains.getV())
    );
  }

  @Override
  public void setTopGains(PIDSVGains gains) {
    topMotor.getConfigurator().apply(
            new Slot0Configs()
                    .withKP(gains.getP())
                    .withKI(gains.getI())
                    .withKD(gains.getD())
                    .withKS(gains.getS())
                    .withKV(gains.getV())
    );
  }

  private void configureHardware() {
    topMotor.setNeutralMode(NEUTRAL_MODE);
    bottomMotor.setNeutralMode(NEUTRAL_MODE);

    topMotor.setInverted(TOP_MOTOR_INVERTED);
    bottomMotor.setInverted(BOTTOM_MOTOR_INVERTED);
  }

  private void configureCurrentLimits() {
    topMotor.getConfigurator().apply(CURRENT_LIMITS_CONFIGS);
    bottomMotor.getConfigurator().apply(CURRENT_LIMITS_CONFIGS);
  }
}
