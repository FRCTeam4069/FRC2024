package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IndexerController;

public class DefualtIndexerCommand extends Command{
    private IndexerController indexer = RobotContainer.indexer;
    boolean isLoaded, shooting, speed;

    double startCurrent = 0;
    

    public DefualtIndexerCommand(BooleanSupplier requested){
        //isLoaded = requested.getAsBoolean();
        addRequirements(RobotContainer.indexer);
        startCurrent = indexer.getCurrent();
    }


    public void execute(){
    //    if(indexer.getCurrent() < 6 || RobotContainer.shooter.isShooting()){
    //         indexer.feedShooter();
    //    }
    //    else if(indexer.getCurrent() >= 6 && !indexer.getPhotoReading()){
    //         indexer.slowFeed();
    //    }   
    //    else{
    //     indexer.stop();
    //    }    

       SmartDashboard.putBoolean("Digital Test", indexer.getPhotoReading());
    }

    public void end(boolean interupted){
        indexer.stop();
    }

    public boolean isFinished(){
        return false;
    }
}
