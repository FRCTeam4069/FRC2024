package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.IntakeController;
import frc.robot.subsystems.IntakeController.positions;

public class AutoLowerIntake extends Command{
    IntakeController i;

    private final double kP = 0.0155, kI = 0, kD = 0;
    private PIDController controller = new PIDController(kP, kI, kD);

    public AutoLowerIntake(IntakeController in){
        
        i = in;
        i.setPosition(positions.LOWER);
        i.ResetEncoder();
    }

    @Override
    public void execute(){
        i.driveArt(controller.calculate(i.getEncoder(), 6));
        SmartDashboard.putNumber("intake pos" , i.getEncoder());
        SmartDashboard.putNumber("Intake Target", 6);
        SmartDashboard.putNumber("Intake Difference", i.getEncoder() - i.getPositionValue());
    }

    @Override
    public void end(boolean interrupted){
        i.stopArt();
    }

    @Override
    public boolean isFinished(){
        return i.getEncoder() - (6) < 5;
    }
    
}
