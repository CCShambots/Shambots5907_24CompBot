package frc.robot.subsystems.intake;

import static frc.robot.Constants.Intake.Hardware.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import frc.robot.ShamLib.motors.talonfx.PIDSVGains;
import frc.robot.ShamLib.motors.talonfx.VelocityTalonFX;
import frc.robot.util.ProxSensor;

public class IntakeIOReal implements IntakeIO {
  protected final VelocityTalonFX topMotor =
      new VelocityTalonFX(TOP_ID, TOP_GAINS.get(), TOP_RATIO);
  protected final VelocityTalonFX bottomMotor =
      new VelocityTalonFX(BOTTOM_ID, TOP_GAINS.get(), BOTTOM_RATIO);

  protected final ProxSensor prox = new ProxSensor(PROX_ID);

  public IntakeIOReal() {
    this(false);
  }

  public IntakeIOReal(boolean sim) {
    if (!sim) configureCurrentLimits();

    configureHardware();

    TOP_GAINS.setOnChange(this::setGains);
  }

  private void configureCurrentLimits() {
    bottomMotor.getConfigurator().apply(CURRENT_LIMIT);
    topMotor.getConfigurator().apply(CURRENT_LIMIT);
  }

  private void configureHardware() {
    topMotor.setNeutralMode(NEUTRAL_MODE);
    bottomMotor.setNeutralMode(NEUTRAL_MODE);

    topMotor.setInverted(TOP_INVERTED);
  }

  @Override
  public void setVoltage(double voltage) {
    topMotor.setVoltage(voltage);
  }

  @Override
  public void setBeltTargetVelocity(double velocity) {
    resetFollower();

    topMotor.setTarget(velocity);
  }

  @Override
  public void resetFollower() {
    bottomMotor.setControl(new Follower(topMotor.getDeviceID(), BOTTOM_INVERTED));
  }

  @Override
  public void stop() {
    topMotor.stopMotor();
    bottomMotor.stopMotor();
  }

  @Override
  public void setGains(PIDSVGains gains) {
    Slot0Configs configs =
        new Slot0Configs()
            .withKP(gains.getP())
            .withKI(gains.getI())
            .withKD(gains.getD())
            .withKS(gains.getS())
            .withKV(gains.getV());

    topMotor.getConfigurator().apply(configs);
    bottomMotor.getConfigurator().apply(configs);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.velocity = topMotor.getEncoderVelocity();
    inputs.targetVelocity = topMotor.getTarget();
    inputs.voltage = topMotor.getMotorVoltage().getValueAsDouble();
    inputs.proxTripped = prox.isActivated();
    inputs.rotorVelocity = topMotor.getRotorVelocity().getValueAsDouble();
  }
}
