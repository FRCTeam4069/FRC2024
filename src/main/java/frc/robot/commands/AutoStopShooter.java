package frc.robot.commands;

import frc.robot.subsystems.ShooterController;

public class AutoStopShooter {
    ShooterController c;

    public AutoStopShooter(ShooterController s){
        c = s;
    }

    public void execute(){
        c.stop();
    }
    public void end(boolean interrupted){
        c.stop();
    }
    public boolean isFinished(){
        return true;
    }
}
