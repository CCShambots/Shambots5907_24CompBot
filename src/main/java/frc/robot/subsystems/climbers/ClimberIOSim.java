package frc.robot.subsystems.climbers;

import static frc.robot.Constants.Climbers.Sim.*;

import frc.robot.ShamLib.motors.talonfx.sim.PhysicsSim;

public class ClimberIOSim extends ClimberIOReal {
  public ClimberIOSim(int id, boolean inverted, int touchSensorPort) {
    super(id, inverted, true, touchSensorPort);

    PhysicsSim.getInstance().addTalonFX(motor, INERTIA);
  }
}
