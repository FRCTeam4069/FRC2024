package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerController;

public class IndexWithSensorCommand extends Command {
    IndexerController ind;

    public IndexWithSensorCommand(IndexerController i){
        ind = i;
    }

    public void execute(){
        ind.feedShooter();
    }

    public void end(boolean interrupted){
        ind.stop();
    }
    public boolean isFinished(){
        return ind.getPhotoReading();
    }
    
}
