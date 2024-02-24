package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterRotationController;

public class ShooterRotationCommand {
    private final ShooterRotationController shooter = RobotContainer.artShooter;

    public ShooterRotationCommand(){

    }

    public void execute(){
        shooter.goToAngle();
    }

    public void end(boolean interrupted){

    }

    public boolean isFinished(){
        return false;
    }
}

