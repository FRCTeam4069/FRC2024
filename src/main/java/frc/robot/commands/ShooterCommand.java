package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterController;

public class ShooterCommand extends Command {
    private ShooterController shooter = RobotContainer.shooter;
    double x , y , theta;

    public ShooterCommand(double x, double theta){
        this.x = x;
        this.y = y;
        this.theta = theta;

        addRequirements(RobotContainer.shooter);
    }

    @Override
    public void execute(){
        //RobotContainer.shooter.ShootWithPos(x, theta); 
        RobotContainer.shooter.drive();
    }

    @Override
    public void end(boolean interupted){
        RobotContainer.shooter.stop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }





    
}
