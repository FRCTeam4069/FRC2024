package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeController;
import frc.robot.subsystems.IntakeController.positions;

public class AutoSetIntakeState extends Command {
   IntakeController intake;
   State state;
   
   public AutoSetIntakeState(IntakeController i, State s){
        intake = i;
        state = s;
        intake.setPosition(positions.LOWER);
   }

    private final double kP = 0.0155, kI = 0, kD = 0;
    private PIDController controller = new PIDController(kP, kI, kD);

   @Override
   public void execute(){
        intake.driveArt(controller.calculate(intake.getEncoder(), intake.getPositionValue()));
        if(state == State.ON) intake.driveFeed();
        else intake.stopFeed();
   }

   @Override
   public void end(boolean interupted){
     if(state == State.ON) intake.driveFeed();
     else intake.stopFeed(); 
   }

   @Override
   public boolean isFinished(){
     //    if(state == State.ON && intake.getFeedSpeed() > 0) return true;
     //    if(state == State.OFF && intake.getFeedSpeed() == 0) return true;
     //    return false;
          return true;
   }

   public enum State{
        ON,
        OFF
    }
}

