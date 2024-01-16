package frc.robot.subsystems.shooter.flywheel;

import static frc.robot.Constants.Flywheel.Hardware.*;

import com.ctre.phoenix6.controls.StrictFollower;
import frc.robot.ShamLib.motors.talonfx.VelocityTalonFX;

public class FlywheelIOReal implements FlywheelIO {
  private final VelocityTalonFX topMotor;
  private final VelocityTalonFX bottomMotor;

  public FlywheelIOReal() {
    topMotor = new VelocityTalonFX(TOP_MOTOR_ID, TOP_MOTOR_GAINS, TOP_MOTOR_RATIO);

    bottomMotor = new VelocityTalonFX(BOTTOM_MOTOR_ID, BOTTOM_MOTOR_GAINS, BOTTOM_MOTOR_RATIO);

    configureCurrentLimits();
    configureHardware();
  }

  @Override
  public void updateInputs(FlywheelInputs inputs) {
    inputs.targetVelocity = topMotor.getTarget();
    inputs.velocity = topMotor.getEncoderVelocity();
  }

  @Override
  public void setFlywheelTarget(double target) {
    topMotor.setTarget(target);
  }

  @Override
  public void setFlywheelVoltage(double voltage) {
    topMotor.setVoltage(voltage);
  }

  private void configureHardware() {
    topMotor.setNeutralMode(NEUTRAL_MODE);
    bottomMotor.setNeutralMode(NEUTRAL_MODE);

    topMotor.setInverted(TOP_MOTOR_INVERTED);
    bottomMotor.setInverted(BOTTOM_MOTOR_INVERTED);

    // I don't know if this would ignore the motor's own motion magic gains
    bottomMotor.setControl(new StrictFollower(TOP_MOTOR_ID));
  }

  private void configureCurrentLimits() {
    topMotor.getConfigurator().apply(CURRENT_LIMITS_CONFIGS);
    bottomMotor.getConfigurator().apply(CURRENT_LIMITS_CONFIGS);
  }
}
