package frc.robot.subsystems.drivetrain;

import static frc.robot.Constants.Drivetrain.Settings.*;
import static frc.robot.Constants.Drivetrain.Hardware.*;

import frc.robot.Constants;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.ShamLib.swerve.DriveCommand;
import frc.robot.ShamLib.swerve.SwerveDrive;
import frc.robot.ShamLib.swerve.TimestampedPoseEstimator;

import java.util.List;
import java.util.function.DoubleSupplier;

public class Drivetrain extends StateMachine<Drivetrain.State> {
    private final SwerveDrive drive;
    private boolean invertPath = false;

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
                () -> invertPath,
                STATE_STD_DEVIATIONS,
                MODULE_1_INFO,
                MODULE_2_INFO,
                MODULE_3_INFO,
                MODULE_4_INFO
        );
    }

    private void registerStateCommands(DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta) {
        registerStateCommand(State.X_SHAPE, () -> drive.setModuleStates(X_SHAPE));

        registerStateCommand(State.IDLE, drive::stopModules);

        registerStateCommand(State.FIELD_ORIENTED_DRIVE, new DriveCommand(
                drive,
                x,
                y,
                theta,
                Constants.Controller.DEADBAND,
                Constants.Controller.DRIVE_CONVERSION,
                this,
                TRAVERSE_SPEED
        ));
    }

    private void registerTransitions() {

    }

    public void addVisionMeasurements(List<TimestampedPoseEstimator.TimestampedVisionUpdate> measurement) {
        drive.addTimestampedVisionMeasurements(measurement);
    }

    @Override
    protected void determineSelf() {
        setState(State.IDLE);
    }

    public enum State {
        UNDETERMINED,
        IDLE,
        X_SHAPE,
        FIELD_ORIENTED_DRIVE,
        FOLLOWING_AUTONOMOUS_TRAJECTORY
    }
}
