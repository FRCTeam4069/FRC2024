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

    positions p;

    public ArticIntakeCommand(positions pos){
        p = pos;
    }

    @Override
    public void execute(){
        
    }

    public enum positions{
        UPPER,
        LOWER
    }
}
