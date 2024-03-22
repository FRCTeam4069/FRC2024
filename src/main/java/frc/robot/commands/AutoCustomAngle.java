package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterController;
import frc.robot.subsystems.ShooterRotationController;
import frc.robot.subsystems.IntakeController.positions;

public class AutoCustomAngle extends Command {

    private ShooterRotationController controller;
    private ShooterController s;
    //private IndexerController indexer;

    ShooterPositions positions;
    double velocity = 0;
    double angle = 0;
    public AutoCustomAngle(ShooterRotationController con, ShooterController rot, ShooterPositions positions){
        this.positions = positions;
        controller = con;
        s = rot;
    }
       
    @Override
    public void initialize() {
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
        else if(positions == ShooterPositions.CLIMB){
            velocity = 70;
            angle = 49.75;
        }

        else{
            velocity = 0;
            angle = 50;
        }
    }
    
    public void execute(){
        
        // else if(positions == ShooterPositions.CLIMB){
        //     velocity = 0;
        //     angle = 3;
        // }
        // else {
        //     velocity = 80;
        //     angle = 62;
        //     //3.2m
        // }

        s.driveWithCustomSpeed(velocity, velocity);
        controller.setCustomAngle(angle);
        SmartDashboard.putBoolean("autoCustomAngle", true);
        SmartDashboard.putNumber("bad angle", angle);
        SmartDashboard.putNumber("bad angle yes", controller.getAngle());


    }

    public void end(boolean interrupted){
        controller.stop();
        SmartDashboard.putBoolean("autoCustomAngle", false);

        //s.stop();
    }
    public boolean isFinished(){
        return MathUtil.isNear(Math.toRadians(angle), controller.getAngle(), Units.degreesToRadians(2))
            && s.atSpeed();
    }
}
