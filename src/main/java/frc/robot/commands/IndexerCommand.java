package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IndexerController;

public class IndexerCommand extends Command{
    private IndexerController indexer = RobotContainer.indexer;
    boolean isLoaded, shooting, speed;
    

    public IndexerCommand(boolean isLoaded, boolean shooting, boolean atSpeed){
        this.isLoaded = isLoaded;
        this.shooting = shooting;
        speed = atSpeed;
    }


    public void execute(){
        if(shooting && speed){
            indexer.feedShooter();
        }

        if(!shooting && !isLoaded){
            indexer.feedShooter();
        }
        else{
            indexer.stop();
        }
    }

    public void end(boolean interupted){
        indexer.stop();
    }

    public boolean isFinished(){
        return false;
    }
}
