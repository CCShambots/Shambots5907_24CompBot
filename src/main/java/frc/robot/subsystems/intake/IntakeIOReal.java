package frc.robot.subsystems.intake;

import static frc.robot.Constants.Intake.Hardware.*;
import static frc.robot.Constants.Intake.Settings.*;

import frc.robot.ShamLib.motors.talonfx.MotionMagicTalonFX;
import frc.robot.ShamLib.motors.talonfx.VelocityTalonFX;
import frc.robot.ShamLib.sensor.ThroughBoreEncoder;

public class IntakeIOReal implements IntakeIO {
  private final MotionMagicTalonFX armMotor =
      new MotionMagicTalonFX(
          ARM_ID, ARM_GAINS, ARM_RATIO, ARM_VELOCITY, ARM_ACCELERATION, ARM_JERK);
  ;
  private final VelocityTalonFX beltMotor = new VelocityTalonFX(BELT_ID, BELT_GAINS, BELT_RATIO);
  ;

  private final ThroughBoreEncoder armEncoder =
      new ThroughBoreEncoder(ARM_ENCODER_ID, ARM_ENCODER_OFFSET);

  public IntakeIOReal() {
    configureHardware();
    syncToAbsoluteEncoder();
  }

  private double getAbsoluteAngle() {
    return armEncoder.getDegrees() * ARM_ENCODER_RATIO;
  }

  private void configureHardware() {
    armMotor.getConfigurator().apply(ARM_CURRENT_LIMIT);
    beltMotor.getConfigurator().apply(BELT_CURRENT_LIMIT);

    armMotor.setNeutralMode(ARM_NEUTRAL_MODE);
    beltMotor.setNeutralMode(BELT_NEUTRAL_MODE);

    armMotor.setInverted(ARM_INVERTED);
    beltMotor.setInverted(BELT_INVERTED);
    armEncoder.setInverted(ARM_ENCODER_INVERTED);
  }

  @Override
  public void setBeltTargetVelocity(double velocity) {
    beltMotor.setTarget(velocity);
  }

  @Override
  public void setArmTargetPosition(double position) {
    armMotor.setTarget(position);
  }

  @Override
  public void stopBelt() {
    beltMotor.stopMotor();
  }

  @Override
  public void stopArm() {
    armMotor.stopMotor();
  }

  @Override
  public void syncToAbsoluteEncoder() {
    armMotor.resetPosition(getAbsoluteAngle());
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.armPosition = armMotor.getEncoderPosition();
    inputs.armTargetPosition = armMotor.getTarget();
    inputs.armVelocity = armMotor.getEncoderVelocity();

    inputs.beltVelocity = beltMotor.getEncoderVelocity();
    inputs.beltTargetVelocity = beltMotor.getTarget();

    inputs.absoluteEncoderPosition = getAbsoluteAngle();
  }
}
