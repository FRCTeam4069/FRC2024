package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterController;
import frc.robot.subsystems.ShooterRotationController;

public class SetShooterCommand extends Command{
    private ShooterController shooter;
    private ShooterRotationController controller;
    private ShooterPositions positions;
    private double velocity = 0;
    private double angle = 0;

    public SetShooterCommand(ShooterController s, ShooterRotationController r, ShooterPositions p){
        shooter = s;
        controller = r;
        positions = p;

        addRequirements(s, r);

        if(positions == ShooterPositions.WALL_AREA){
            velocity = 60;
            angle = 35;
        } 
        else if(positions == ShooterPositions.WHITE_LINE){
            velocity = 70;
            angle = 50;
        }
        else {
            velocity = 80;
            angle = 60;
        }
    }

    public void execute(){
        shooter.driveWithCustomSpeed(velocity, velocity/2);
        controller.setCustomAngle(angle);
    }

    public void end(boolean interrupted){
        shooter.stop();
    }

    public boolean inFinished(){
        return false;
    }
}
