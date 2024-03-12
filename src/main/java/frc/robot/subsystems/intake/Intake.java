package frc.robot.subsystems.intake;

import static frc.robot.Constants.Intake.Settings.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.ShamLib.WhileDisabledInstantCommand;
import frc.robot.ShamLib.motors.tuning.LinearTuningCommand;
import org.littletonrobotics.junction.Logger;

public class Intake extends StateMachine<Intake.State> {
  private final IntakeIO io;

  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io, Trigger tuningInc, Trigger tuningDec, Trigger tuningStop) {
    super("Intake", State.UNDETERMINED, State.class);

    this.io = io;

    new WaitCommand(2)
        .andThen(new WhileDisabledInstantCommand(() -> io.resetFollower()))
        .schedule();

    io.updateInputs(inputs);

    registerStateCommands(tuningInc, tuningDec, tuningStop);
    registerTransitions();
  }

  private void registerStateCommands(Trigger tuningInc, Trigger tuningDec, Trigger tuningStop) {
    registerStateCommand(
        State.IDLE, new SequentialCommandGroup(new InstantCommand(io::stop), watchProxCommand()));
    registerStateCommand(
        State.INTAKE,
        new SequentialCommandGroup(
            new InstantCommand(() -> io.setBeltTargetVelocity(BELT_SPEED)), watchProxCommand()));
    registerStateCommand(
        State.EXPEL,
        new SequentialCommandGroup(
            new InstantCommand(() -> io.setBeltTargetVelocity(-BELT_SPEED)), watchProxCommand()));

    registerStateCommand(
        State.VOLTAGE_CALC,
        new SequentialCommandGroup(
            new LinearTuningCommand(
                tuningStop,
                tuningInc,
                tuningDec,
                io::setVoltage,
                () -> inputs.rotorVelocity,
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

  private Command watchProxCommand() {
    return new RunCommand(
        () -> {
          if (inputs.proxTripped) {
            setFlag(State.PROX_TRIPPED);
          } else {
            clearFlag(State.PROX_TRIPPED);
          }
        });
  }

  public boolean ringPresent() {
    return inputs.proxTripped;
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
    VOLTAGE_CALC,

    // flags
    PROX_TRIPPED
  }
}
