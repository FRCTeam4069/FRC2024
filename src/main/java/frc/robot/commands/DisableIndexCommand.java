package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerController;

public class DisableIndexCommand extends Command {
    IndexerController ind;

    public DisableIndexCommand(IndexerController i){
        ind = i;
        
    }

    public void initialize(){
        ind.stop();
    }

    public void execute(){
        
    }



    public boolean isFinished(){
        return true;
    }
    
}
