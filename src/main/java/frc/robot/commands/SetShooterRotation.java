package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterController;
import frc.robot.subsystems.ShooterRotationController;
import frc.robot.subsystems.ShooterController.SlotConfigs;

public class SetShooterRotation extends Command {
    private ShooterRotationController controller;
    private ShooterController s;
    DoubleSupplier d;

    public SetShooterRotation(ShooterRotationController c, DoubleSupplier distance, ShooterController v){
        controller = c;
        d = distance;
        this.s = v;
        addRequirements(c);

    }

    public void execute(){
            
            double angle =  190.478 * (Math.pow((d.getAsDouble()* 100 / 2.54), 0.0787374)) - 218.00;
            SmartDashboard.putNumber("Set Angle", angle);
            controller.setCustomAngle(angle);
            if(angle > 65){
                s.driveWithCustomSpeed(90, 90);
            }
            else if(angle < 65 && angle > 40){
                s.driveWithCustomSpeed(70, 70);
            }
            else{
                s.driveWithCustomSpeed(60, 60);
            }
         SmartDashboard.putNumber("Adjusted X", angle);
        
       
    }

    public void end(boolean interrupted){
        s.stop();
    }

    public boolean isFinished(){
        return false;
    }

}
