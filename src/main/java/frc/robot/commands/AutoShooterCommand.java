package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IndexerController;
import frc.robot.subsystems.RevBlinkinPatterns;
import frc.robot.subsystems.ShooterController;
import frc.robot.subsystems.ShooterRotationController;

public class AutoShooterCommand extends Command {
    private ShooterRotationController controller;
    private ShooterController s;
    //private IndexerController indexer;
    private ShooterPositions positions;
    double d;

    double startTime = 0;

    public AutoShooterCommand(ShooterRotationController c, ShooterController v, IndexerController i, ShooterPositions p){
        controller = c;
        //indexer = i;
        this.s = v;
        positions = p;
        addRequirements(c);
        startTime = Timer.getFPGATimestamp();

        addRequirements(c, v);

    }
    double velocity = 0, angle = 0;

    @Override
    public void execute(){
        if(positions == ShooterPositions.WALL_AREA){
            velocity = 60; //60 from wall
            angle = 35;    //35 from wall
        } 
        else if(positions == ShooterPositions.WHITE_LINE){
            velocity = 65;
            angle = 45;
        }
        else if(positions == ShooterPositions.SAFE_ZONE){
            velocity = 0;
            angle = 60;
        }
        else{
            velocity = 0;
            angle = 50;
        }
        // else if(positions == ShooterPositions.CLIMB){
        //     velocity = 0;
        //     angle = 3;
        // }
        // else {
        //     velocity = 80;
        //     angle = 62;
        //     //3.2m
        // }

        s.driveWithCustomSpeed(velocity, velocity/2);
        controller.setCustomAngle(angle);
    }

    @Override
    public void end(boolean interrupted){
        //controller.goToAngle
        //indexer.feedShooter();
    }

    @Override
    public boolean isFinished(){
        return controller.atPosition();
    }
}
