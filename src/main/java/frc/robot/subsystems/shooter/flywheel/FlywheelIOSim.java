package frc.robot.subsystems.shooter.flywheel;

import static frc.robot.Constants.Flywheel.Sim.*;

import frc.robot.ShamLib.motors.talonfx.sim.PhysicsSim;

public class FlywheelIOSim extends FlywheelIOReal {
  public FlywheelIOSim() {
    super(true);

    PhysicsSim.getInstance().addTalonFX(topMotor, TOP_INERTIA);
    PhysicsSim.getInstance().addTalonFX(bottomMotor, BOTTOM_INERTIA);
  }
}
