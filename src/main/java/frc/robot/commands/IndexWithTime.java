package frc.robot.commands;

import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerController;

public class IndexWithTime extends Command {
    IndexerController indexer;
    double startTime = 0;
    double endTime = 0;
    double speed = 0.65;

    public IndexWithTime(IndexerController c, double time){
        indexer = c;
        endTime=time;
        startTime = Timer.getFPGATimestamp();
    }

    public IndexWithTime(IndexerController c, double time, double speed){
        indexer = c;
        endTime=time;
        startTime = Timer.getFPGATimestamp();
        this.speed = speed;
    }

    @Override
    public void execute(){
        indexer.setCustomSpeed(speed);
    }

    public void end(){
        indexer.stop();
    }

    public boolean isFinished(){
        return Timer.getFPGATimestamp() - startTime > endTime;
    }
    
}
