package frc.robot.subsystems.intake;

import static frc.robot.Constants.Intake.Sim.*;

import frc.robot.ShamLib.motors.talonfx.sim.PhysicsSim;

public class IntakeIOSim extends IntakeIOReal {
  public IntakeIOSim() {
    super(true);

    PhysicsSim.getInstance().addTalonFX(bottomMotor, BELT_INERTIA);
    PhysicsSim.getInstance().addTalonFX(topMotor, BELT_INERTIA);
  }
}
