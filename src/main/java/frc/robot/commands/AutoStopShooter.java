package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterController;

public class AutoStopShooter extends Command {
    ShooterController c;

    public AutoStopShooter(ShooterController s){
        c = s;
    }

    @Override
    public void execute(){
        c.stop();
    }

    @Override
    public void end(boolean interrupted){
        c.stop();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
