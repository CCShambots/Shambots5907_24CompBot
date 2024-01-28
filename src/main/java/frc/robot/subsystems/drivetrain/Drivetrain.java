package frc.robot.subsystems.drivetrain;

import static frc.robot.Constants.Drivetrain.Settings.*;
import static frc.robot.Constants.Drivetrain.Hardware.*;

import frc.robot.Constants;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.ShamLib.swerve.SwerveDrive;

public class Drivetrain extends StateMachine<Drivetrain.State> {
    private final SwerveDrive drive;

    public Drivetrain() {
        super("Drivetrain", State.UNDETERMINED, State.class);

        drive = new SwerveDrive(
                Constants.currentBuildMode,
                GYRO_ID,
                MODULE_DRIVE_GAINS,
                MODULE_TURN_GAINS,
                MAX_CHASSIS_SPEED,
                MAX_CHASSIS_ACCELERATION,
                MAX_CHASSIS_ROTATIONAL_SPEED,
                MAX_CHASSIS_ROTATIONAL_ACCELERATION,
                MAX_MODULE_TURN_SPEED,
                MAX_MODULE_TURN_ACCELERATION,
                AUTO_THETA_GAINS,
                AUTO_TRANSLATION_GAINS,
                false,
                MODULE_CAN_BUS,
                GYRO_CAN_BUS,
                CURRENT_LIMITS_CONFIGS,
                this,
                true,
                //change
                () -> false,
                STATE_STD_DEVIATIONS,
                MODULE_1_INFO,
                MODULE_2_INFO,
                MODULE_3_INFO,
                MODULE_4_INFO
        );
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
