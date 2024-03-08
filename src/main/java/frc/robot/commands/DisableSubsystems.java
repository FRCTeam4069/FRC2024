package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerController;
import frc.robot.subsystems.IntakeController;
import frc.robot.subsystems.ShooterController;
import frc.robot.subsystems.ShooterRotationController;

public class DisableSubsystems extends Command{
    ShooterRotationController ar;
    ShooterController sh;
    IndexerController in;
    IntakeController it;

    public DisableSubsystems(ShooterRotationController a, ShooterController s, IndexerController i, IntakeController ir ){
        ar = a;
        sh = s;
        in = i;
        it = ir;
    }

    public void initialize(){
        ar.stop();
        sh.stop();
        in.stop();
        it.stopFeed();
        it.stopArt();
    }

    public boolean isFinished(){
        return true;
    }
    
}
