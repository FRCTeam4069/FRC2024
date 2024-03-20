package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerController;
import frc.robot.subsystems.ShooterController;

public class DefualtShooter extends Command {
    IndexerController controlelr;
    BooleanSupplier isShooting, requested;

    public DefualtShooter(IndexerController c, BooleanSupplier i, BooleanSupplier r){
        isShooting = i;
        requested = r;
        controlelr = c;
    }

    public void execute(){
        if(!requested.getAsBoolean() && isShooting.getAsBoolean()){
            controlelr.feedShooter();
        }
        else{
             controlelr.stop();
        }
        
    }

    public void end(boolean interupted){
        controlelr.stop();
    }

    public boolean isFnished(){
        return false;
    }
    
}
