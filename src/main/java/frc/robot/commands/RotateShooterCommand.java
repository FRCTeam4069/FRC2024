package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterRotationController;

public class RotateShooterCommand extends Command {
    private ShooterRotationController controller;
    double angle;
    double velocity = 0; 

    public RotateShooterCommand(ShooterRotationController c, double angle){
        controller = c;
        this.angle = angle;

        addRequirements(c);

    }

    @Override
    public void execute(){
        controller.setCustomAngle(angle);
    }

    @Override
    public void end(boolean interrupted){
        //controller.goToAngle
        //indexer.feedShooter();
        controller.stop();
    }

    @Override
    public boolean isFinished(){
        return MathUtil.isNear(Math.toRadians(angle), controller.getAngle(), 0.05);
    }
}
