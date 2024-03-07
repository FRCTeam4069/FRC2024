package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterRotationController;

public class ClimberCommand extends Command{
    private ClimberSubsystem c;
    private DoubleSupplier CSpeed;
    private ShooterRotationController fs;

    public ClimberCommand(ClimberSubsystem s, DoubleSupplier sp){
        c = s;
        CSpeed = sp;
        addRequirements(s);
        //fs = f;
    }



    public void execute(){
         
        //SmartDashboard.putNumber("Climber Value", c.getEncoder());
        SmartDashboard.putNumber("Climber Current", c.getCurrent());
        if(Math.abs(CSpeed.getAsDouble()) > 0.5){
           
            c.setPower(CSpeed.getAsDouble());
            // SmartDashboard.putNumber("Climber Speed", CSpeed.getAsDouble());
            // SmartDashboard.putNumber("Climber Pos", c.getEncoder());
            //fs.setCustomAngle(0);
        }
        else{
            c.setPower(0);
            // SmartDashboard.putNumber("Climber Speed",c.getPower());
            // SmartDashboard.putNumber("Climber Pos", c.getEncoder());
        }
       
    }

    // public void end(boolean interrupted){
    //     c.setPower(0);
    // }

    public boolean isFinished(){
        return false;
    }
}
