package frc.robot.subsystems.intake.commands;

import static frc.robot.Constants.Intake.Settings.*;
import static frc.robot.Constants.doubleEqual;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class StowCommand extends Command {
    private final Intake intake;
    private final boolean expel;
    private final Timer expelTimer = new Timer();

    public StowCommand(Intake intake, boolean expel) {
        this.intake = intake;
        this.expel = expel;
    }

    @Override
    public void initialize() {
        expelTimer.restart();

        if (expel) {
            intake.getIO().setBeltTargetVelocity(-BELT_SPEED); //TODO: add configuration for this
        }
        else {
            intake.getIO().setBeltTargetVelocity(0); //TODO: change this to cut power instead of set setpoint
        }

        intake.getIO().setArmTargetPosition(STOW_ANGLE);
    }

    @Override
    public void execute() {
        //stop expelling if designated time has elapsed and we haven't already stopped expelling (to avoid spamming can)
        if (expelTimer.hasElapsed(STOW_EXPEL_DURATION) && doubleEqual(0, intake.getInputs().beltTargetVelocity)) {
            intake.getIO().setBeltTargetVelocity(0); //TODO: change this to cut power instead of set setpoint
            expelTimer.stop();
        }

        //resync motor to absolute encoder if needed
        if (motorAtSetpoint() != encoderAtSetpoint()/* && intake.getInputs().armvelocity < whatever*/) {
            intake.getIO().syncToAbsoluteEncoder();
        }
    }

    private boolean motorAtSetpoint() {
        return doubleEqual(intake.getInputs().armPosition, STOW_ANGLE, ANGLE_SETPOINT_TOLERANCE);
    }

    private boolean encoderAtSetpoint() {
        return doubleEqual(intake.getInputs().absoluteEncoderPosition, STOW_ANGLE, ANGLE_SETPOINT_TOLERANCE);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return motorAtSetpoint() && encoderAtSetpoint();
    }
}
