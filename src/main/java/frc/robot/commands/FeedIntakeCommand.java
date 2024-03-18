package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeController;

public class FeedIntakeCommand extends Command{
    private IntakeController intake = RobotContainer.intake;

    public FeedIntakeCommand(){
        //addRequirements(RobotContainer.intake);
    }

    @Override
    public void execute(){
        intake.driveFeed();
    }

    @Override

    public void end(boolean interupted){
        intake.stopFeed();
    }

    @Override

    public boolean isFinished(){
        return false;
    }
} 
