package frc.robot.subsystems.drivetrain;

import frc.robot.ShamLib.SMF.StateMachine;

public class Drivetrain extends StateMachine<Drivetrain.State> {

    public Drivetrain() {
        super("Drivetrain", State.UNDETERMINED, State.class);
    }

    @Override
    protected void determineSelf() {
        setState(State.IDLE);
    }

    public enum State {
        UNDETERMINED,
        IDLE,
        SOFT_E_STOP
    }
}
