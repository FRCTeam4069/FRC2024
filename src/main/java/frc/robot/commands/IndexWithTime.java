package frc.robot.commands;

import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerController;

public class IndexWithTime extends Command {
    IndexerController indexer;
    double startTime = 0;
    double endTime = 0;

    public IndexWithTime(IndexerController c, double time){
        indexer = c;
        endTime=time;
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute(){
        indexer.feedShooter();
    }

    public void end(){
        indexer.stop();
    }

    public boolean isFinished(){
        return Timer.getFPGATimestamp() - startTime > endTime;
    }
    
}
