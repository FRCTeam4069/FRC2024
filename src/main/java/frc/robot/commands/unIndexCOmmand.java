package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerController;

public class unIndexCOmmand extends Command{
    
    IndexerController indexer;

    public unIndexCOmmand(IndexerController i){
        indexer = i;
        addRequirements(i);
    }

    public void execute(){
        indexer.unFeedShooter();
    }
    public void end(boolean interupted){
        indexer.stop();
    }
    public boolean isFinished(){
        return false;
    }
}
