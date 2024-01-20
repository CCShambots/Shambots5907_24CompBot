package frc.robot.subsystems.indexer;

import static frc.robot.Constants.Indexer.Hardware.*;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.ShamLib.motors.talonfx.VelocityTalonFX;

public class IndexerIOReal implements IndexerIO {
  protected final VelocityTalonFX beltMotor =
      new VelocityTalonFX(BELT_MOTOR_ID, BELT_GAINS, BELT_RATIO);

  private final DigitalInput[] proxSensors = {
    new DigitalInput(PROX_1_ID), new DigitalInput(PROX_2_ID), new DigitalInput(PROX_3_ID)
  };

  public IndexerIOReal() {
    this(false);
  }

  public IndexerIOReal(boolean sim) {
    if (sim) {
      configureCurrentLimits();
    }

    configureHardware();
  }

  private void configureCurrentLimits() {
    beltMotor.getConfigurator().apply(BELT_MOTOR_CURRENT_LIMIT);
  }

  private void configureHardware() {
    beltMotor.setInverted(INVERT_BELT_MOTOR);
    beltMotor.setNeutralMode(BELT_MOTOR_NEUTRAL_MODE);
  }

  @Override
  public void stop() {
    beltMotor.stopMotor();
  }

  @Override
  public void setTargetVelocity(double targetVelocity) {
    beltMotor.setTarget(targetVelocity);
  }

  @Override
  public void setVoltage(double voltage) {
    beltMotor.setVoltage(voltage);
  }

  @Override
  public void updateInputs(IndexerInputs inputs) {
    for (int i = 0; i < proxSensors.length; i++) {
      inputs.proximitySensors[i] = proxSensors[i].get();
    }

    inputs.beltVelocity = beltMotor.getEncoderVelocity();
    inputs.beltTargetVelocity = beltMotor.getTarget();
  }
}
