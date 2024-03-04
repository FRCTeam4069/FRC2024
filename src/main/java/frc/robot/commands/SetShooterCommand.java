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

        

        //v from wall = 60
        //a from wall = 35

        //v from white line = 60
        //a from white line = 48.5

        //v from safe zone/ equavilet distance = 80
        //a from safe zone/ equavilet distance = 58

        //32 inches from safe velocity = 70; 
        //angle = 63;

        //64 inches from safe
        //velocity = 70;    
        //angle = 65;

        //behind redline velocity = 85; 
        //angle = 71.5;

        //19 inches from centerstage same as wall

    }

    public void execute(){
        if(positions == ShooterPositions.WALL_AREA){
            velocity = 85; //60 from wall
            angle = 71.5;    //35 from wall
        } 
        else if(positions == ShooterPositions.WHITE_LINE){
            velocity = 60;
            angle = 48.5;
        }
        else if(positions == ShooterPositions.AMP_AREA){
            velocity = 9.85;
            angle = 31;
        }
        else if(positions == ShooterPositions.CLIMB){
            velocity = 0;
            angle = 10;
        }
        else {
            velocity = 80;
            angle = 58;
            //3.2m
        }

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
