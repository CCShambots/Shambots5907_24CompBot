package frc.robot.subsystems.indexer;

import static frc.robot.Constants.Indexer.Settings.*;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.ShamLib.SMF.StateMachine;
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
                new InstantCommand(() -> io.setTargetVelocity(INDEX_SPEED)),
                new WaitUntilCommand(() -> inputs.prox2),
                new InstantCommand(io::stop))
            .withTimeout(INDEX_TIMEOUT)
            .andThen(
                new ConditionalCommand(
                    transitionCommand(State.HOLDING_RING),
                    transitionCommand(State.LOST_RING),
                    () -> inputs.prox1 && inputs.prox2)));

    registerStateCommand(
        State.EXPECT_RING_FRONT,
        new SequentialCommandGroup(
                new InstantCommand(() -> io.setTargetVelocity(-EXPECT_SPEED)),
                new WaitUntilCommand(() -> inputs.prox2),
                new InstantCommand(() -> io.setTargetVelocity(-INDEX_SPEED)),
                new WaitUntilCommand(() -> inputs.prox1),
                new InstantCommand(io::stop))
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

    addTransition(State.EXPECT_RING_FRONT, State.LOST_RING);
    addTransition(State.EXPECT_RING_BACK, State.LOST_RING);

    addTransition(State.EXPECT_RING_BACK, State.HOLDING_RING);
    addTransition(State.EXPECT_RING_FRONT, State.HOLDING_RING);

    addTransition(State.HOLDING_RING, State.FEED_TO_SHOOTER);
    addTransition(State.HOLDING_RING, State.PASS_THROUGH);
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
    PASS_THROUGH,
    FEED_TO_SHOOTER,
    CLEANSE,
    LOST_RING
  }
}
