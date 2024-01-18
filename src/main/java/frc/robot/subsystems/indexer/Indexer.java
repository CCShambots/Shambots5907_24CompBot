package frc.robot.subsystems.indexer;

import frc.robot.ShamLib.SMF.StateMachine;
import org.littletonrobotics.junction.Logger;

public class Indexer extends StateMachine<Indexer.State> {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  public Indexer(IndexerIO io) {
    super("Indexer", State.UNDETERMINED, State.class);

    this.io = io;
  }

  @Override
  protected void determineSelf() {}

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
    SOFT_E_STOP,
    MANUAL_CONTROL
  }
}
