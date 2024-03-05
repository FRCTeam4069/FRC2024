package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterController;
import frc.robot.subsystems.ShooterRotationController;

public class SetShooterRotation extends Command {
    private ShooterRotationController controller;
    private ShooterController s;
    double d;

    public SetShooterRotation(ShooterRotationController c, double distance, ShooterController v){
        controller = c;
        d = distance;
        this.s = v;
        addRequirements(c);

    }

    public void execute(){
            
            double angle = 190.478 * Math.pow(((RobotContainer.FrontCamera.getXDistanceToApriltag(7, 1)) * 100 / 2.54), 0.0787374) - 219.781;
            SmartDashboard.putNumber("Set Angle", angle);
            controller.setCustomAngle(angle);
            if(angle > 75){
                s.driveWithCustomSpeed(85, 85);
            }
            else if(angle < 75 && angle > 40){
                s.driveWithCustomSpeed(70, 70);
            }
            else{
                s.driveWithCustomSpeed(60, 60);
            }
            
        
       
    }

    public void end(boolean interrupted){
        s.stop();
    }

    public boolean isFinished(){
        return false;
    }

}
