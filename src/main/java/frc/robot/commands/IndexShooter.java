package frc.robot.commands;

import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerController;

public class IndexShooter extends Command{
    private IndexerController indexer;
    private double startTime = 0;

    public IndexShooter(IndexerController i){
        indexer = i;
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute(){
        indexer.feedShooter();
    }
    @Override
    public boolean isFinished(){
        return Timer.getFPGATimestamp() - startTime > 3;
    }
    @Override
    public void end(boolean interrupted){
        indexer.stop();
    }
    
}
