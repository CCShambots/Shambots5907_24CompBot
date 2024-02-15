package frc.robot.subsystems.shooter.arm;

import static frc.robot.Constants.Arm.Sim.*;

import frc.robot.ShamLib.motors.talonfx.sim.PhysicsSim;

public class ArmIOSim extends ArmIOReal {
  public ArmIOSim() {
    super(true);

    PhysicsSim.getInstance().addTalonFX(leaderMotor, LEADER_INERTIA);
    PhysicsSim.getInstance().addTalonFX(followerMotor, FOLLOWER_INERTIA);
  }

  @Override
  public void updateInputs(ArmInputs inputs) {
    inputs.encoderPosition = leaderMotor.getEncoderPosition();
    inputs.motorPosition = leaderMotor.getEncoderPosition();
    inputs.targetPosition = leaderMotor.getTarget();
    inputs.motorVelocity = leaderMotor.getEncoderVelocity();
    inputs.motorVoltage = leaderMotor.getMotorVoltage().getValueAsDouble();
    inputs.motorRotorVelocity = leaderMotor.getRotorVelocity().getValueAsDouble();

    inputs.followerPosition = followerMotor.getEncoderPosition();
    inputs.followerVelocity = followerMotor.getEncoderVelocity();
  }

  @Override
  public void syncToAbsoluteEncoder() {}
}
