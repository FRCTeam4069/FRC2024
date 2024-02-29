package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IndexerController;

public class DefualtIndexerCommand extends Command{
    private IndexerController indexer = RobotContainer.indexer;
    boolean isLoaded, shooting, speed;
    

    public DefualtIndexerCommand(BooleanSupplier requested){
        //isLoaded = requested.getAsBoolean();
        addRequirements(RobotContainer.indexer);
    }


    public void execute(){
        if(indexer.getPhotoReading() < 0.15 || RobotContainer.shooter.isShooting()){
            indexer.feedShooter();
        }
        else{
            indexer.stop();
            this.end(false);
        }
    }

    public void end(boolean interupted){
        indexer.stop();
    }

    public boolean isFinished(){
        return false;
    }
}
