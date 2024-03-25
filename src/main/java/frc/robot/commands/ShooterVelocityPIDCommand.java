package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterController;
import frc.robot.subsystems.IntakeController.positions;

public class ShooterVelocityPIDCommand extends Command {
    private double velocity;
    private ShooterController shooter;
    private PIDController pid = new PIDController(0.11, 0.48, 0.01);

    public ShooterVelocityPIDCommand(ShooterController shooter, double velocity) {
        this.velocity = velocity;
        this.shooter = shooter;

    }

    @Override
    public void initialize() {
        pid.setTolerance(4);
        
    }

    @Override
    public void execute() {
        // shooter.setSpeed(pid.calculate(shooter.getFasterVelocity(), velocity));
        
    }
    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }
    @Override
    public void end(boolean interrupted) {

    }
    
}
