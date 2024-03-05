package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeController;

public class AutoSetIntakeState extends Command {
   IntakeController intake;
   State state;
   
   public AutoSetIntakeState(IntakeController i, State s){
        intake = i;
        state = s;
   }

   public void execute(){
        if(state == State.ON) intake.driveFeed();
        else intake.stopFeed();
   }

   public void end(boolean interupted){

   }
   public boolean isFinished(){
        if(state == State.ON && intake.getFeedSpeed() > 0) return true;
        if(state == State.OFF && intake.getFeedSpeed() == 0) return true;
        return false;
   }

   public enum State{
        ON,
        OFF
    }
}

