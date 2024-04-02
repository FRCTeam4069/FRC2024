package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerController;

public class SimpleIndexerCommand extends Command {
    private IndexerController indexer;
    private boolean ringInside = false;
    private boolean lastReading = false;
    public SimpleIndexerCommand(IndexerController indexer) {
        this.indexer = indexer;
    }

    @Override
    public void initialize() {
        lastReading = indexer.getPhotoReading();
    }

    @Override
    public void execute() {
        boolean currentReading = indexer.getPhotoReading();
        if (currentReading && !lastReading) {
            ringInside = true;
        }

        indexer.feedShooter();

        lastReading = currentReading;

    }

    @Override
    public boolean isFinished() {
        return ringInside;
    }

    @Override
    public void end(boolean interrupted) {
        indexer.stop();
        
    }

    
}
