package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeController;
import frc.robot.subsystems.IntakeController.positions;

public class BringIntakeUpCommand extends Command {
    private IntakeController intakeController;

    private final double kP = 0.0155, kI = 0, kD = 0;
    private PIDController controller = new PIDController(kP, kI, kD);

    public BringIntakeUpCommand(IntakeController i){
        intakeController = i;
        addRequirements(i);
        intakeController.setPosition(positions.UPPER);
    }

    public void execute(){
        
        intakeController.driveArt(controller.calculate(intakeController.getEncoder(), intakeController.getPositionValue()));
    }

    public void end(boolean interrupted){

    }
    public boolean isFinished(){
        return false;
    }
}
