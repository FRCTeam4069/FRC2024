package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeController;

public class BackIntakeCommand extends Command {
    IntakeController intake;

     public BackIntakeCommand(IntakeController i){
        intake = i;
        addRequirements(i);
    }

    @Override
    public void execute(){
        intake.backIntake();
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
