package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.lights.Lights;
import frc.robot.subsystems.shooter.Shooter;

public class DetermineRingStatusCommand extends Command {
  private final Shooter shooter;
  private final Indexer indexer;
  private final Lights lights;

  private boolean isFinished = false;

  public DetermineRingStatusCommand(Shooter shooter, Indexer indexer, Lights lights) {
    this.indexer = indexer;
    this.shooter = shooter;
    this.lights = lights;
  }

  @Override
  public void initialize() {
    isFinished = false;

    switch (indexer.getState()) {
      case HOLDING_RING -> {
        shooter.requestTransition(Shooter.State.PARTIAL_STOW);
        lights.requestTransition(Lights.State.HAVE_RING);

        isFinished = true;
      }
      case LOST_RING -> {
        shooter.requestTransition(Shooter.State.PASS_THROUGH);
        indexer.requestTransition(Indexer.State.CLEANSE);
        lights.requestTransition(Lights.State.EJECT);
      }
      case INDEXING -> {
        shooter.requestTransition(Shooter.State.PARTIAL_STOW);
        lights.requestTransition(Lights.State.HAVE_RING);
      }
      default -> {
        indexer.requestTransition(Indexer.State.IDLE);
        shooter.requestTransition(Shooter.State.PARTIAL_STOW);
        lights.requestTransition(Lights.State.NO_RING);

        isFinished = true;
      }
    }
  }

  @Override
  public void execute() {
    if (indexer.getState() == Indexer.State.PASS_THROUGH
        && indexer.isFlag(Indexer.State.PROX_CLEAR)) {
      lights.requestTransition(Lights.State.NO_RING);
      indexer.requestTransition(Indexer.State.IDLE);
      shooter.requestTransition(Shooter.State.PARTIAL_STOW);

      isFinished = true;
    }
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
