package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
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
    }

    public void execute(){
        i.driveArt(controller.calculate(i.getEncoder(), i.getPositionValue()));
    }
    public void end(boolean interrupted){
        i.stopArt();
    }
    public boolean isFinished(){
        return i.getEncoder() - i.getPositionValue() < 3;
    }
    
}
