package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.IntakeController;

public class ArticIntakeCommand extends Command{
    private IntakeController intake = RobotContainer.intake;
    private final double LOWER_VAL = IntakeConstants.LOWER_POSITION - 3, 
                         UPPER_VAL = IntakeConstants.UPPER_POSITION - 3;

    private final double kP = 0, kI = 0, kD = 0;
    private PIDController controller = new PIDController(kP, kI, kD);

    private double target = 0;

    positions p;

    public ArticIntakeCommand(positions pos){
        p = pos;

        if(p == positions.UPPER) target = UPPER_VAL;
        else target = LOWER_VAL; 
    }

    @Override
    public void execute(){
        intake.driveArt(controller.calculate(intake.getEncoder(), target));
    }

    public boolean isFinished(){
        return Math.abs(intake.getEncoder()) - Math.abs(target) < 2;
    }
    public void end(boolean interupted){
        intake.driveArt(0);
    }

    public enum positions{
        UPPER,
        LOWER
    }
}
