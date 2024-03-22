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
    double spinMulitplier = 0.75;
    double angleTolerance = 1.5;
    double velocityTolerance = 2.0;
    public CustomShooterCommand(ShooterRotationController con, ShooterController rot, double velocity, double angle){
        controller = con;
        s = rot;
        this.velocity = velocity;
        this.angle = angle;
    }

    /**
     * Sets the speed and angle of the shooter
     * @param con
     * @param rot
     * @param velocity
     * @param angle 90 is parallel with the ground, 0 is perp
     * @param spinMulitplier the multiplier used to slow down a motor so it spins
     */
    public CustomShooterCommand(ShooterRotationController con, ShooterController rot, double velocity, double angle, double spinMulitplier){
        controller = con;
        s = rot;
        this.velocity = velocity;
        this.angle = angle;
        this.spinMulitplier = spinMulitplier;
    }

    public CustomShooterCommand(ShooterRotationController con, ShooterController rot, double velocity, double angle, double spinMulitplier, double angleTolerance){
        controller = con;
        s = rot;
        this.velocity = velocity;
        this.angle = angle;
        this.spinMulitplier = spinMulitplier;
        this.angleTolerance = angleTolerance;
    }

    public CustomShooterCommand(ShooterRotationController con, ShooterController rot, double velocity, double angle, double spinMulitplier, double angleTolerance, double velocityTolerance){
        controller = con;
        s = rot;
        this.velocity = velocity;
        this.angle = angle;
        this.spinMulitplier = spinMulitplier;
        this.angleTolerance = angleTolerance;
        this.velocityTolerance = velocityTolerance;
    }
       
    @Override
    public void initialize() {

    }
    
    public void execute(){
        s.driveWithCustomSpeed(velocity, velocity*spinMulitplier);
        controller.setCustomAngle(angle);

    }

    public void end(boolean interrupted){
        controller.stop();
        //s.stop();
    }
    public boolean isFinished(){
        return MathUtil.isNear(Math.toRadians(angle), controller.getAngle(), Units.degreesToRadians(angleTolerance)) && s.atSpeed(velocityTolerance);
    }
}
