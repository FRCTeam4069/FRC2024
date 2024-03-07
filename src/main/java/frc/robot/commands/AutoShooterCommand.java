package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IndexerController;
import frc.robot.subsystems.ShooterController;
import frc.robot.subsystems.ShooterRotationController;

public class AutoShooterCommand extends Command {
    private ShooterRotationController controller;
    private ShooterController s;
    private IndexerController indexer;
    double d;

    public AutoShooterCommand(ShooterRotationController c, ShooterController v, IndexerController i){
        controller = c;
        indexer = i;
        this.s = v;
        addRequirements(c);

    }

    public void execute(){
            // if(indexer.getPhotoReading()){
            //     double angle = 190.478 * Math.pow((((RobotContainer.FrontCamera.getXDistanceToApriltag(4, 7))) * 100 / 2.54), 0.0787374) - 219.781;
            //     SmartDashboard.putNumber("Set Angle", angle);
            //     controller.setCustomAngle(angle);
            //     if(angle > 75){
            //         s.driveWithCustomSpeed(85, 85);
            //     }
            //     else if(angle < 75 && angle > 40){
            //         s.driveWithCustomSpeed(70, 70);
            //     }
            //     else{
            //         s.driveWithCustomSpeed(60, 60);
            //     }
            //     SmartDashboard.putNumber("Adjusted X", d);
            // }
           
        
       
    }

    public void end(boolean interrupted){
        //controller.goToAngle();
    }

    public boolean isFinished(){
        return !indexer.getPhotoReading();
    }
}
