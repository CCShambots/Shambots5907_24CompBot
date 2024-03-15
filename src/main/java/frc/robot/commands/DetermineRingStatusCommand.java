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
        // If we're holding a ring, set the shooter to stow again and make sure the lights show we
        // have a ring
        shooter.requestTransition(Shooter.State.STOW);
        lights.requestTransition(Lights.State.HAVE_RING);

        isFinished = true;
      }
      case LOST_RING -> {
        // Clear the robot of any potential ring if the indexer has lost it
        /*shooter.requestTransition(Shooter.State.PASS_THROUGH);
        indexer.requestTransition(Indexer.State.CLEANSE);
        lights.requestTransition(Lights.State.EJECT);*/
        lights.requestTransition(Lights.State.ERROR);
        isFinished = true;
      }
      case INDEXING -> {
        // Indicate we have a ring and stow the shooter, but wait since the robot isn't done
        // indexing yet
        shooter.requestTransition(Shooter.State.STOW);
        lights.requestTransition(Lights.State.HAVE_RING);

        isFinished = true;
      }
      default -> {
        indexer.requestTransition(Indexer.State.IDLE);
        shooter.requestTransition(Shooter.State.STOW);
        lights.requestTransition(Lights.State.NO_RING);

        isFinished = true;
      }
    }

    shooter.transitionCommand(Shooter.State.STOW);
  }

  @Override
  public void execute() {
    /*if (indexer.getState() == Indexer.State.PASS_THROUGH
        && indexer.isFlag(Indexer.State.PROX_CLEAR)) {
      lights.requestTransition(Lights.State.NO_RING);
      indexer.requestTransition(Indexer.State.IDLE);
      shooter.requestTransition(Shooter.State.PARTIAL_STOW);

      isFinished = true;
    }*/
    // ????
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
