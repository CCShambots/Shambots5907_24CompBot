package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double beltVelocity = 0.0; // RPM
    public double beltTargetVelocity = 0.0; // RPM

    public double armPosition = 0.0; // DEG
    public double armTargetPosition = 0.0; // DEG
    public double armVelocity = 0.0; //DEG/s

    public double absoluteEncoderPosition = 0.0; // DEG
  }

  public void setBeltTargetVelocity(double velocity);

  public void setArmTargetPosition(double position);

  public void syncToAbsoluteEncoder();

  public void updateInputs(IntakeIOInputs inputs);
}
