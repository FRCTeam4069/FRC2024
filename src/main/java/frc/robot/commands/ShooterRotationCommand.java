package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterRotationController;

public class ShooterRotationCommand extends Command {
    private ShooterRotationController shooter;

    public ShooterRotationCommand(ShooterRotationController c){
        shooter = c;
        
        addRequirements(c);
    }

    public void execute(){
        if(!shooter.isClimbing()){
            shooter.goToAngle();
        }
        else{
            shooter.setCustomAngle(15);
        }
        
    }

    public void end(boolean interrupted){

    }

    public boolean isFinished(){
        return false;
    }
}

