package frc.robot.subsystems.intake;

import static frc.robot.Constants.Intake.Sim.*;

import frc.robot.ShamLib.motors.talonfx.sim.PhysicsSim;

import java.util.function.BooleanSupplier;

public class IntakeIOSim extends IntakeIOReal {
  private final BooleanSupplier proxSupplier;

  public IntakeIOSim(BooleanSupplier proxSupplier) {
    super(true);

    this.proxSupplier = proxSupplier;

    PhysicsSim.getInstance().addTalonFX(bottomMotor, BELT_INERTIA);
    PhysicsSim.getInstance().addTalonFX(topMotor, BELT_INERTIA);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.velocity = topMotor.getEncoderVelocity();
    inputs.targetVelocity = topMotor.getTarget();
    inputs.voltage = topMotor.getMotorVoltage().getValueAsDouble();
    inputs.proxTripped = proxSupplier.getAsBoolean();
  }
}
