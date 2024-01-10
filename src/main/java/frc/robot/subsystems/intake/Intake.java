package frc.robot.subsystems.intake;

import static frc.robot.Constants.Intake.Settings.*;
import static frc.robot.Constants.doubleEqual;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ShamLib.SMF.StateMachine;
import org.littletonrobotics.junction.Logger;

public class Intake extends StateMachine<Intake.State> {
  private final IntakeIO io;

  private Timer syncTimeout = new Timer();
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    super("Intake", State.UNDETERMINED, State.class);

    this.io = io;

    io.updateInputs(inputs);

    registerStateCommands();
    registerTransitions();
  }

  private void registerStateCommands() {
    registerStateCommand(State.SEEKING_STOWED, new SequentialCommandGroup());
  }

  private void registerTransitions() {}

  private boolean needsSync() {
    //arm motor position and abs encoder position are outside of tolerance as well as making sure the arm isn't moving very fast
    return !doubleEqual(inputs.armPosition, inputs.absoluteEncoderPosition, AUTO_SYNC_TOLERANCE) && doubleEqual(inputs.armVelocity, 0.0, 1.0);
  }

  public IntakeIO getIO() {
    return io;
  }

  public IntakeIO.IntakeIOInputs getInputs() {
    return inputs;
  }

  @Override
  protected void update() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);

    //sync the motor to the absolute encoder if needed (and feature is enabled)
    //there is a configurable timer so we don't spam the can network
    if (USE_AUTO_SYNC && needsSync() && syncTimeout.hasElapsed(MINIMUM_TIME_BETWEEN_SYNC_ATTEMPTS)) {
      io.syncToAbsoluteEncoder();
      syncTimeout.restart();
    }
  }

  @Override
  protected void determineSelf() {
    // TODO: MAKE THIS ACTUALLY DO THE THING
    setState(State.SOFT_E_STOP);
  }

  public enum State {
    UNDETERMINED,
    STOWED,
    SEEKING_STOWED,
    DEPLOYED,
    SEEKING_DEPLOYED,
    SOFT_E_STOP,
    MANUAL_CONTROL
  }
}
