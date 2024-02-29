package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterRotationController;

public class SetShooterRotation extends Command {
    private ShooterRotationController controller;
    double d;

    public SetShooterRotation(ShooterRotationController c, double distance){
        controller = c;
        d = distance;
        addRequirements(c);

    }

    public void execute(){
        double angle = Math.pow((190.478 * (d)), 0.0787374) - 219.781;
        controller.setCustomAngle(angle);
    }

    public void end(boolean interrupted){

    }

    public boolean isFinished(){
        return false;
    }

}
