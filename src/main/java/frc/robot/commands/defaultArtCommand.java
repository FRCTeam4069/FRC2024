package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeController;
import frc.robot.subsystems.IntakeController.positions;

public class defaultArtCommand extends Command {

    private final IntakeController intake = RobotContainer.intake;

    public defaultArtCommand(){
        addRequirements(RobotContainer.intake);    
    }

    private final double kP = 0.0155, kI = 0, kD = 0;
    private PIDController controller = new PIDController(kP, kI, kD);


    public void execute(){
        intake.driveArt(controller.calculate(intake.getEncoder(), intake.getPositionValue()));
    
    }

    public void end(boolean interupted){
        intake.stopFeed();
        //intake.setPosition(positions.UPPER);
    }
    public boolean isFinished(){
        return false;
    }


    
}
