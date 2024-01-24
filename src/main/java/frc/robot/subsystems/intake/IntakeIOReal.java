package frc.robot.subsystems.intake;

import static frc.robot.Constants.Intake.Hardware.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import frc.robot.ShamLib.motors.talonfx.PIDSVGains;
import frc.robot.ShamLib.motors.talonfx.VelocityTalonFX;

public class IntakeIOReal implements IntakeIO {
  protected final VelocityTalonFX topMotor = new VelocityTalonFX(TOP_ID, TOP_GAINS.get(), TOP_RATIO);
  protected final VelocityTalonFX bottomMotor =
      new VelocityTalonFX(BOTTOM_ID, BOTTOM_GAINS.get(), BOTTOM_RATIO);

  public IntakeIOReal() {
    this(false);
  }

  public IntakeIOReal(boolean sim) {
    if (!sim) configureCurrentLimits();

    configureHardware();

    TOP_GAINS.setOnChange(this::setTopGains);
    BOTTOM_GAINS.setOnChange(this::setBottomGains);
  }

  private void configureCurrentLimits() {
    bottomMotor.getConfigurator().apply(CURRENT_LIMIT);
    topMotor.getConfigurator().apply(CURRENT_LIMIT);
  }

  private void configureHardware() {
    topMotor.setNeutralMode(NEUTRAL_MODE);

    topMotor.setInverted(TOP_INVERTED);
    bottomMotor.setInverted(BOTTOM_INVERTED);
  }

  @Override
  public void setTopVoltage(double voltage) {
    topMotor.setVoltage(voltage);
  }

  @Override
  public void setBottomVoltage(double voltage) {
    bottomMotor.setVoltage(voltage);
  }

  @Override
  public void setBeltTargetVelocity(double velocity) {
    topMotor.setTarget(velocity);
    bottomMotor.setTarget(velocity);
  }

  @Override
  public void stop() {
    topMotor.stopMotor();
    bottomMotor.stopMotor();
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
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.topVelocity = topMotor.getEncoderVelocity();
    inputs.topTargetVelocity = topMotor.getTarget();
    inputs.topVoltage = topMotor.getMotorVoltage().getValueAsDouble();

    inputs.bottomVelocity = bottomMotor.getEncoderVelocity();
    inputs.bottomTargetVelocity = bottomMotor.getTarget();
    inputs.bottomVoltage = bottomMotor.getMotorVoltage().getValueAsDouble();
  }
}
