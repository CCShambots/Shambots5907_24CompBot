package frc.robot.subsystems.intake;

import static frc.robot.Constants.Intake.Settings.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ShamLib.SMF.StateMachine;
import org.littletonrobotics.junction.Logger;

public class Intake extends StateMachine<Intake.State> {
  private final IntakeIO io;

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
