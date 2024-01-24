package frc.robot.subsystems.indexer;

import static frc.robot.Constants.Indexer.Hardware.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.ShamLib.motors.talonfx.PIDSVGains;
import frc.robot.ShamLib.motors.talonfx.VelocityTalonFX;

public class IndexerIOReal implements IndexerIO {
  protected final VelocityTalonFX beltMotor =
      new VelocityTalonFX(BELT_MOTOR_ID, BELT_GAINS.get(), BELT_RATIO);

  private final DigitalInput prox1 = new DigitalInput(PROX_1_ID);
  private final DigitalInput prox2 = new DigitalInput(PROX_2_ID);

  public IndexerIOReal() {
    this(false);
  }

  public IndexerIOReal(boolean sim) {
    if (sim) {
      configureCurrentLimits();
    }

    configureHardware();

    BELT_GAINS.setOnChange(this::setGains);
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
  public void setGains(PIDSVGains gains) {
    beltMotor.getConfigurator().apply(
            new Slot0Configs()
                    .withKP(gains.getP())
                    .withKI(gains.getI())
                    .withKD(gains.getD())
                    .withKS(gains.getS())
                    .withKV(gains.getV())
    );
  }

  @Override
  public void updateInputs(IndexerInputs inputs) {
    inputs.prox1 = prox1.get();
    inputs.prox2 = prox2.get();

    inputs.beltVelocity = beltMotor.getEncoderVelocity();
    inputs.beltTargetVelocity = beltMotor.getTarget();
  }
}
