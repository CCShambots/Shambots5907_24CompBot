package frc.robot.subsystems.indexer;

import static frc.robot.Constants.Indexer.Sim.*;

import frc.robot.ShamLib.motors.talonfx.sim.PhysicsSim;

import java.util.function.BooleanSupplier;

public class IndexerIOSim extends IndexerIOReal {
    private final BooleanSupplier prox1;
    private final BooleanSupplier prox2;
    private final BooleanSupplier prox3;

    public IndexerIOSim(BooleanSupplier prox1, BooleanSupplier prox2, BooleanSupplier prox3) {
        super(true);

        this.prox1 = prox1;
        this.prox2 = prox2;
        this.prox3 = prox3;

        PhysicsSim.getInstance().addTalonFX(beltMotor, MOTOR_INERTIA);
    }

    @Override
    public void updateInputs(IndexerInputs inputs) {
        inputs.beltVelocity = beltMotor.getEncoderVelocity();
        inputs.beltTargetVelocity = beltMotor.getTarget();
        inputs.beltVoltage = beltMotor.getMotorVoltage().getValueAsDouble();

        inputs.prox1 = prox1.getAsBoolean();
        inputs.prox2 = prox2.getAsBoolean();
        inputs.prox3 = prox3.getAsBoolean();
    }
}
