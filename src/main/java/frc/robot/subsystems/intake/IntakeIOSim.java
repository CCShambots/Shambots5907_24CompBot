package frc.robot.subsystems.intake;

import static frc.robot.Constants.Intake.Sim.*;

import frc.robot.ShamLib.motors.talonfx.sim.PhysicsSim;

public class IntakeIOSim extends IntakeIOReal {
  public IntakeIOSim() {
    super(true);

    PhysicsSim.getInstance().addTalonFX(armMotor, ARM_INERTIA);
    PhysicsSim.getInstance().addTalonFX(beltMotor, BELT_INERTIA);

    armMotor.resetPosition(ARM_START_ANGLE);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.armVelocity = armMotor.getEncoderVelocity();

    inputs.armPosition = armMotor.getEncoderPosition();
    inputs.absoluteEncoderPosition = armMotor.getEncoderPosition();

    inputs.armTargetPosition = armMotor.getTarget();

    inputs.beltVelocity = beltMotor.getEncoderVelocity();
    inputs.beltTargetVelocity = beltMotor.getTarget();
    inputs.beltPosition = beltMotor.getEncoderPosition();
  }
}
