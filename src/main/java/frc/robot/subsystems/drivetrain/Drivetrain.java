package frc.robot.subsystems.drivetrain;

import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.Constants;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.ShamLib.swerve.SwerveDrive;

public class Drivetrain extends StateMachine<Drivetrain.State> {
    //private final SwerveDrive drive;

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
