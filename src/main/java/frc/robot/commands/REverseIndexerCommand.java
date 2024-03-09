package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerController;

public class REverseIndexerCommand extends Command{
    IndexerController i;

    BooleanSupplier requested, PastSensor, atSensor;

    public REverseIndexerCommand(IndexerController in, BooleanSupplier PastSensor, BooleanSupplier atSensor){
        //this.requested = requested;
        this. atSensor = atSensor;
        this.PastSensor = PastSensor;
        i = in;
    }

    public void execute(){
        if(PastSensor.getAsBoolean() && !i.getPhotoReading()){
            i.slowFeed();
        }
        else{
            i.stop();
            this.end(false);
        }
        
    }

    public void end(boolean interrupted){
        i.stop();
    }

    public boolean isFinished(){
        return i.getPhotoReading();
    }
}
