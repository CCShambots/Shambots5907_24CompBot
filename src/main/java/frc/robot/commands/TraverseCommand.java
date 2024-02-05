package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class TraverseCommand extends Command {
    private final Shooter shooter;
    private final Indexer indexer;

    public TraverseCommand(Shooter shooter, Indexer indexer) {
        this.indexer = indexer;
        this.shooter = shooter;
    }

    @Override
    public void initialize() {
        switch (indexer.getState()) {
            case HOLDING_RING -> {}
            case LOST_RING -> {
                shooter.requestTransition(Shooter.State);
                indexer.requestTransition(Indexer.State.CLEANSE);
            }
        }
    }
}
