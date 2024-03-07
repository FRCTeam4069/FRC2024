package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterController;
import frc.robot.subsystems.ShooterRotationController;

public class AutoStopShooter extends Command {
    ShooterController c;
    ShooterRotationController s;

    public AutoStopShooter(ShooterController s, ShooterRotationController f){
        c = s;
        this.s = f;
    }

    @Override
    public void execute(){
        c.stop();
        s.stop();
    }

    @Override
    public void end(boolean interrupted){
        c.stop();
        s.stop();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
