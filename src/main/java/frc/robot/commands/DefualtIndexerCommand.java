package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IndexerController;
import frc.robot.subsystems.LEDController.Colours;

public class DefualtIndexerCommand extends Command{
    private IndexerController indexer = RobotContainer.indexer;
    boolean shooting, speed;
    private boolean isLoaded = false;
    private DoubleSupplier rt, lt;

    private double start = 0;

    double startCurrent = 0;
    
    boolean something = false; // quinn's name not mine


    public DefualtIndexerCommand(BooleanSupplier requested, DoubleSupplier l, DoubleSupplier r){
        //isLoaded = requested.getAsBoolean();
        addRequirements(RobotContainer.indexer);
        startCurrent = indexer.getCurrent();
        lt = l;
        rt = r;
    }
    int i = 0;


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
        isLoaded = indexer.getPhotoReading();

        if(isLoaded){
            something = true;
        }
        if(RobotContainer.shooter.isShooting()){
            indexer.setCustomSpeed(-rt.getAsDouble());
            //something = false;
        }

        if(!isLoaded){ 
            indexer.feedShooter();
        }
        if(isLoaded){
            indexer.setCustomSpeed(lt.getAsDouble()/8);
           // RobotContainer.led.setColour(Colours.STARTUPPATTERN);
            //i = 0;
        //this.end(false);
        }   
        // else{
        //     something = false;
        // }
       SmartDashboard.putBoolean("Digital Test", isLoaded);
       //SmartDashboard.putBoolean("Digital Something", something);
    }

    public void end(boolean interupted){
        indexer.stop();
    }

    public boolean isFinished(){
        return false;
    }
}
