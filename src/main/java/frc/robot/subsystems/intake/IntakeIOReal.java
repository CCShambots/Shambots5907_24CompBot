package frc.robot.subsystems.intake;

import static frc.robot.Constants.Intake.Hardware.*;

import frc.robot.ShamLib.motors.talonfx.VelocityTalonFX;

public class IntakeIOReal implements IntakeIO {
  protected final VelocityTalonFX topMotor = new VelocityTalonFX(TOP_ID, TOP_GAINS, TOP_RATIO);
  protected final VelocityTalonFX bottomMotor =
      new VelocityTalonFX(BOTTOM_ID, BOTTOM_GAINS, BOTTOM_RATIO);

  public IntakeIOReal() {
    this(false);
  }

  public IntakeIOReal(boolean sim) {
    if (!sim) configureCurrentLimits();

    configureHardware();
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
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.topVelocity = topMotor.getEncoderVelocity();
    inputs.topTargetVelocity = topMotor.getTarget();
    inputs.topVoltage = topMotor.getMotorVoltage().getValueAsDouble();

    inputs.bottomVelocity = bottomMotor.getEncoderVelocity();
    inputs.bottomTargetVelocity = bottomMotor.getTarget();
    inputs.bottomVoltage = bottomMotor.getMotorVoltage().getValueAsDouble();
  }
}
