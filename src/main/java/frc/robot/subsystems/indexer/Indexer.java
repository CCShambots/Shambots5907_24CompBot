package frc.robot.subsystems.indexer;

import static frc.robot.Constants.Indexer.Settings.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ShamLib.SMF.StateMachine;
import frc.robot.ShamLib.motors.tuning.LinearTuningCommand;
import java.util.concurrent.atomic.AtomicBoolean;
import org.littletonrobotics.junction.Logger;

public class Indexer extends StateMachine<Indexer.State> {
  private final IndexerIO io;
  private final IndexerInputsAutoLogged inputs = new IndexerInputsAutoLogged();

  public Indexer(IndexerIO io, Trigger tuningInc, Trigger tuningDec, Trigger tuningStop) {
    super("Indexer", State.UNDETERMINED, State.class);

    this.io = io;

    registerStateCommands(tuningInc, tuningDec, tuningStop);
    registerTransitions();
  }

  private void registerStateCommands(Trigger tuningInc, Trigger tuningDec, Trigger tuningStop) {
    registerStateCommand(
        State.EXPECT_RING_BACK,
        new SequentialCommandGroup(
            new InstantCommand(() -> io.setTargetVelocity(EXPECT_SPEED)), // Spin up
            new WaitUntilCommand(() -> inputs.prox1), // Wait for back prox to be tripped
            transitionCommand(State.INDEXING))); // Start indexing

    registerStateCommand(
        State.EXPECT_RING_FRONT,
        new SequentialCommandGroup(
            new InstantCommand(() -> io.setTargetVelocity(-EXPECT_SPEED)), // Spin up in reverse
            new WaitUntilCommand(() -> inputs.prox2), // Wait for front prox to be tripped
            transitionCommand(State.INDEXING))); // Start indexing

    registerStateCommand(
        State.INDEXING,
        indexCommand() // index
            .withTimeout(
                INDEX_TIMEOUT) // Stop it after some time in case it oscillates or something
            .andThen(
                new ConditionalCommand( // transition to holding ring or lost ring depending on if
                    // first two prox sensors are tripped and last one is not
                    transitionCommand(State.HOLDING_RING),
                    transitionCommand(State.LOST_RING),
                    () -> inputs.prox1 && inputs.prox2 && !inputs.prox3)));

    registerStateCommand(
        State.PASS_THROUGH,
        new SequentialCommandGroup(
            new InstantCommand(() -> io.setTargetVelocity(PASS_THROUGH_SPEED)), // spin up
            new WaitUntilCommand(
                () -> !(inputs.prox1 || inputs.prox2)), // wait for both prox to be untripped
            new InstantCommand(io::stop), // stop motor
            transitionCommand(State.IDLE))); // go to idle

    registerStateCommand(State.IDLE, io::stop);

    registerStateCommand(
        State.FEED_TO_SHOOTER,
        new SequentialCommandGroup(
            new InstantCommand(() -> io.setTargetVelocity(FEED_SPEED)), // spin up
            new WaitUntilCommand(
                () -> !(inputs.prox1 || inputs.prox2)), // wait for neither prox to be tripped
            transitionCommand(State.IDLE))); // go to idle

    registerStateCommand(
        State.CLEANSE,
        new ParallelCommandGroup(
            new InstantCommand(() -> io.setTargetVelocity(PASS_THROUGH_SPEED)),
            proxClearCommand()));

    registerStateCommand(
        State.VOLTAGE_CALC,
        new LinearTuningCommand(
            tuningStop,
            tuningInc,
            tuningDec,
            io::setVoltage,
            () -> inputs.beltRotorVelocity,
            () -> inputs.beltVoltage,
            VOLTAGE_INCREMENT));

    registerStateCommand(State.IDLE, idleWatchCommand());

    registerStateCommand(State.HOLDING_RING, () -> io.stop());
    registerStateCommand(
        State.LOST_RING,
        () -> {
          io.stop();
          requestTransition(State.INDEXING);
        });
  }

  private void registerTransitions() {
    addOmniTransition(State.IDLE, () -> io.stop());
    addOmniTransition(State.CLEANSE);

    addTransition(State.IDLE, State.HOLDING_RING);
    addTransition(State.IDLE, State.INDEXING);

    addTransition(State.IDLE, State.EXPECT_RING_BACK);
    addTransition(State.IDLE, State.EXPECT_RING_FRONT);

    addTransition(State.INDEXING, State.LOST_RING);
    addTransition(State.INDEXING, State.HOLDING_RING);

    addTransition(State.HOLDING_RING, State.FEED_TO_SHOOTER);
    addTransition(State.HOLDING_RING, State.PASS_THROUGH);

    addTransition(State.EXPECT_RING_BACK, State.INDEXING);
    addTransition(State.EXPECT_RING_FRONT, State.INDEXING);

    addTransition(State.IDLE, State.VOLTAGE_CALC);
  }

  private Command indexCommand() {
    AtomicBoolean isFinished = new AtomicBoolean(false);

    return new FunctionalCommand(
        () -> isFinished.set(false),
        () -> {
          // either first two are set and we have ring or both are unset and we lost ring (assuming
          // 3rd is unset)
          if (inputs.prox1 == inputs.prox2 && !inputs.prox3) isFinished.set(true);

          // avoid doing stuff if its already done
          if (!isFinished.get()) {
            // none of the prox are set (where did ring go)
            if (!inputs.prox1 && !inputs.prox2 && !inputs.prox3) isFinished.set(true);

            // 2nd or 3rd prox on, reverse
            else if (inputs.prox3 || inputs.prox2) io.setTargetVelocity(-INDEX_SPEED);

            // otherwise forward
            else io.setTargetVelocity(INDEX_SPEED);
          }
        },
        (interrupted) -> io.stop(),
        isFinished::get);
  }

  private Command idleWatchCommand() {
    return new RunCommand(
        () -> {
          if (inputs.prox1 && inputs.prox2 && !inputs.prox3) {
            requestTransition(State.HOLDING_RING);
          } else if (inputs.prox1 || inputs.prox2 || inputs.prox3) {
            requestTransition(State.INDEXING);
          }
        });
  }

  private Command proxClearCommand() {
    return new RunCommand(
        () -> {
          if (!(inputs.prox1 || inputs.prox2 || inputs.prox3)) {
            setFlag(State.PROX_CLEAR);
          } else {
            clearFlag(State.PROX_CLEAR);
          }
        });
  }

  public boolean ringPresent() {
    return getState() == State.HOLDING_RING || getState() == State.INDEXING;
  }

  public boolean isProx1Active() {
    return inputs.prox1;
  }

  public boolean isProx2Active() {
    return inputs.prox2;
  }

  public boolean isProx3Active() {
    return inputs.prox3;
  }

  @Override
  protected void determineSelf() {
    setState(State.IDLE);
  }

  @Override
  protected void update() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
  }

  public enum State {
    UNDETERMINED,
    HOLDING_RING,
    IDLE,
    EXPECT_RING_FRONT,
    EXPECT_RING_BACK,
    INDEXING,
    PASS_THROUGH,
    FEED_TO_SHOOTER,
    CLEANSE,
    LOST_RING,
    VOLTAGE_CALC,

    // flags
    PROX_CLEAR
  }
}
