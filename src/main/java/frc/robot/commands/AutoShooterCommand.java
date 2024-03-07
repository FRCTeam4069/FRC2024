package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IndexerController;
import frc.robot.subsystems.RevBlinkinPatterns;
import frc.robot.subsystems.ShooterController;
import frc.robot.subsystems.ShooterRotationController;

public class AutoShooterCommand extends Command {
    private ShooterRotationController controller;
    private ShooterController s;
    //private IndexerController indexer;
    private ShooterPositions positions;
    double d;

    double startTime = 0;

    public AutoShooterCommand(ShooterRotationController c, ShooterController v, IndexerController i, ShooterPositions p){
        controller = c;
        //indexer = i;
        this.s = v;
        positions = p;
        addRequirements(c);
        startTime = Timer.getFPGATimestamp();

        addRequirements(c, v);

    }
    double velocity = 0, angle = 0;

    @Override
    public void execute(){
        
        controller.setCustomAngle(57);;
        SmartDashboard.putNumber("SHooter Hell: ", angle);
        SmartDashboard.putNumber("Fucking Shooter angle", controller.getAngle());
    }

    @Override
    public void end(boolean interrupted){
        //controller.goToAngle
        //indexer.feedShooter();
        controller.stop();
    }

    @Override
    public boolean isFinished(){
        return MathUtil.isNear(Math.toRadians(57), controller.getAngle(), 0.05);
    }
}
