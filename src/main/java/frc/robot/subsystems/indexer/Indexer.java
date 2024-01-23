package frc.robot.subsystems.indexer;

import static frc.robot.Constants.Indexer.Settings.*;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.ShamLib.SMF.StateMachine;
import java.util.concurrent.atomic.AtomicBoolean;
import org.littletonrobotics.junction.Logger;

public class Indexer extends StateMachine<Indexer.State> {
  private final IndexerIO io;
  private final IndexerInputsAutoLogged inputs = new IndexerInputsAutoLogged();

  public Indexer(IndexerIO io) {
    super("Indexer", State.UNDETERMINED, State.class);

    this.io = io;

    registerStateCommands();
    registerTransitions();
  }

  private void registerStateCommands() {
    registerStateCommand(
        State.EXPECT_RING_BACK,
        new SequentialCommandGroup(
            new InstantCommand(() -> io.setTargetVelocity(EXPECT_SPEED)),
            new WaitUntilCommand(() -> inputs.prox1),
            transitionCommand(State.INDEXING)));

    registerStateCommand(
        State.EXPECT_RING_FRONT,
        new SequentialCommandGroup(
            new InstantCommand(() -> io.setTargetVelocity(-EXPECT_SPEED)),
            new WaitUntilCommand(() -> inputs.prox2),
            transitionCommand(State.INDEXING)));

    registerStateCommand(
        State.INDEXING,
        indexCommand()
            .withTimeout(INDEX_TIMEOUT)
            .andThen(
                new ConditionalCommand(
                    transitionCommand(State.HOLDING_RING),
                    transitionCommand(State.LOST_RING),
                    () -> inputs.prox1 && inputs.prox2)));

    registerStateCommand(
        State.PASS_THROUGH,
        new SequentialCommandGroup(
            new InstantCommand(() -> io.setTargetVelocity(PASS_THROUGH_SPEED)),
            new WaitUntilCommand(() -> !(inputs.prox1 || inputs.prox2)),
            transitionCommand(State.IDLE)));

    registerStateCommand(State.IDLE, io::stop);

    registerStateCommand(
        State.FEED_TO_SHOOTER,
        new SequentialCommandGroup(
            new InstantCommand(() -> io.setTargetVelocity(FEED_SPEED)),
            new WaitUntilCommand(() -> !(inputs.prox1 || inputs.prox2)),
            transitionCommand(State.IDLE)));

    registerStateCommand(State.CLEANSE, () -> io.setTargetVelocity(PASS_THROUGH_SPEED));
  }

  private void registerTransitions() {
    addOmniTransition(State.IDLE);
    addOmniTransition(State.CLEANSE);

    addTransition(State.INDEXING, State.LOST_RING);
    addTransition(State.INDEXING, State.HOLDING_RING);

    addTransition(State.HOLDING_RING, State.FEED_TO_SHOOTER);
    addTransition(State.HOLDING_RING, State.PASS_THROUGH);

    addTransition(State.EXPECT_RING_BACK, State.INDEXING);
    addTransition(State.EXPECT_RING_FRONT, State.INDEXING);
  }

  private Command indexCommand() {
    AtomicBoolean isFinished = new AtomicBoolean(false);

    return new FunctionalCommand(
        () -> isFinished.set(false),
        () -> {
          // either both are set and we have ring or both are unset and we lost ring
          if (inputs.prox1 == inputs.prox2) isFinished.set(true);

          // if first prox is on but second isnt, run forwards
          else if (inputs.prox1) io.setTargetVelocity(INDEX_SPEED);

          // otherwise vice versa
          else io.setTargetVelocity(-INDEX_SPEED);
        },
        (interrupted) -> io.stop(),
        isFinished::get);
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
    LOST_RING
  }
}
