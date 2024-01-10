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
      intake.getIO().setBeltTargetVelocity(-BELT_SPEED);
    } else {
      intake
          .getIO()
          .setBeltTargetVelocity(0);
    }

    intake.getIO().setArmTargetPosition(STOW_ANGLE);
  }

  @Override
  public void execute() {
    // stop expelling if designated time has elapsed and we haven't already stopped expelling (to
    // avoid spamming can)
    if (expelTimer.hasElapsed(STOW_EXPEL_DURATION)
        && doubleEqual(0, intake.getInputs().beltTargetVelocity)) {
      intake
          .getIO()
          .setBeltTargetVelocity(0);
      expelTimer.stop();
    }
  }

  private boolean motorAtSetpoint() {
    return doubleEqual(intake.getInputs().armPosition, STOW_ANGLE, ANGLE_SETPOINT_TOLERANCE);
  }

  @Override
  public void end(boolean interrupted) {
    expelTimer.stop();
    intake.getIO().setBeltTargetVelocity(0);
  }

  @Override
  public boolean isFinished() {
    //arm made it to top and expelling is done
    return motorAtSetpoint() && doubleEqual(intake.getInputs().beltTargetVelocity, 0);
  }
}
