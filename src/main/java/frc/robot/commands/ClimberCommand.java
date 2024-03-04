package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommand extends Command{
    private ClimberSubsystem c;
    private DoubleSupplier CSpeed;

    public ClimberCommand(ClimberSubsystem s, DoubleSupplier sp){
        c = s;
        CSpeed = sp;
        addRequirements(s);
    }

    public void execute(){
        c.setPower(CSpeed.getAsDouble());
    }

    // public void end(boolean interrupted){
    //     c.setPower(0);
    // }

    public boolean isFinished(){
        return false;
    }
}
