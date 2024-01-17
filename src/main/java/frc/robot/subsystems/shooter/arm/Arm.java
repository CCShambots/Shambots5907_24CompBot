package frc.robot.subsystems.shooter.arm;

import frc.robot.ShamLib.SMF.StateMachine;

public class Arm extends StateMachine<Arm.State> {


    public Arm() {
        super("Shooter Arm", State.UNDETERMINED, State.class);
    }

    @Override
    protected void determineSelf() {

    }

    public enum State {
        UNDETERMINED
    }
}
