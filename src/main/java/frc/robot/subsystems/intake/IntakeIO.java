package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double beltVelocity = 0.0; // RPM
    public double beltTargetVelocity = 0.0; // RPM

    public double armPosition = 0.0; // RAD
    public double armTargetPosition = 0.0; // RAD
  }

  public void setBeltTargetVelocity(double velocity);

  public void setArmTargetVelocity(double velocity);

  public void updateInputs(IntakeIOInputs inputs);
}
