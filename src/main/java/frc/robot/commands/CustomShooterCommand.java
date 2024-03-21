package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterController;
import frc.robot.subsystems.ShooterRotationController;
import frc.robot.subsystems.IntakeController.positions;

public class CustomShooterCommand extends Command {

    private ShooterRotationController controller;
    private ShooterController s;
    //private IndexerController indexer;

    double velocity;
    double angle;
    public CustomShooterCommand(ShooterRotationController con, ShooterController rot, double velocity, double angle){
        controller = con;
        s = rot;
        this.velocity = velocity;
        this.angle = angle;
    }
       
    @Override
    public void initialize() {

    }
    
    public void execute(){
        s.driveWithCustomSpeed(velocity, velocity/2);
        controller.setCustomAngle(angle);

    }

    public void end(boolean interrupted){
        controller.stop();
        //s.stop();
    }
    public boolean isFinished(){
        return MathUtil.isNear(Math.toRadians(angle), controller.getAngle(), Units.degreesToRadians(2));
    }
}
