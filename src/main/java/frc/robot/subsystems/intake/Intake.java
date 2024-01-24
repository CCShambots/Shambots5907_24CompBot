package frc.robot.subsystems.intake;

import static frc.robot.Constants.Intake.Settings.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.ShamLib.motors.tuning.LinearTuningCommand;
import org.littletonrobotics.junction.Logger;

public class Intake extends StateMachine<Intake.State> {
  private final IntakeIO io;

  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io, Trigger tuningInc, Trigger tuningDec, Trigger tuningStop) {
    super("Intake", State.UNDETERMINED, State.class);

    this.io = io;

    io.updateInputs(inputs);

    registerStateCommands(tuningInc, tuningDec, tuningStop);
    registerTransitions();
  }

  private void registerStateCommands(Trigger tuningInc, Trigger tuningDec, Trigger tuningStop) {
    registerStateCommand(State.IDLE, io::stop);
    registerStateCommand(State.INTAKE, () -> io.setBeltTargetVelocity(BELT_SPEED));
    registerStateCommand(State.EXPEL, () -> io.setBeltTargetVelocity(-BELT_SPEED));

    registerStateCommand(
        State.VOLTAGE_CALC,
        new SequentialCommandGroup(
            new LinearTuningCommand(
                tuningStop,
                tuningInc,
                tuningDec,
                io::setVoltage,
                () -> inputs.velocity,
                () -> inputs.voltage,
                VOLTAGE_INC),
            transitionCommand(State.IDLE)));
  }

  private void registerTransitions() {
    addOmniTransition(State.IDLE);
    addOmniTransition(State.INTAKE);
    addOmniTransition(State.EXPEL);

    addTransition(State.IDLE, State.VOLTAGE_CALC);
  }

  @Override
  protected void update() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
  }

  @Override
  protected void determineSelf() {
    // wait for rc to orchestrate things
    io.resetFollower();
    setState(State.IDLE);
  }

  public enum State {
    UNDETERMINED,
    IDLE,
    INTAKE,
    EXPEL,
    VOLTAGE_CALC
  }
}
