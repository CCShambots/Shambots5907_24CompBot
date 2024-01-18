package frc.robot.subsystems.intake;

import static frc.robot.Constants.Intake.Settings.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.ShamLib.SMF.StateMachine;
import org.littletonrobotics.junction.Logger;

public class Intake extends StateMachine<Intake.State> {
  private final IntakeIO io;

  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    super("Intake", State.UNDETERMINED, State.class);

    this.io = io;

    io.updateInputs(inputs);

    registerStateCommands();
    registerTransitions();
  }

  private void registerStateCommands() {
    registerStateCommand(State.IDLE, io::stop);
    registerStateCommand(State.INTAKE, () -> io.setBeltTargetVelocity(BELT_SPEED));
    registerStateCommand(State.EXPEL, () -> io.setBeltTargetVelocity(-BELT_SPEED));
  }

  private void registerTransitions() {
    addOmniTransition(State.IDLE);
    addOmniTransition(State.INTAKE);
    addOmniTransition(State.EXPEL);
  }

  @Override
  protected void update() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
  }

  @Override
  protected void determineSelf() {
    // wait for rc to orchestrate things
    setState(State.IDLE);
  }

  public enum State {
    UNDETERMINED,
    IDLE,
    INTAKE,
    EXPEL
  }
}
