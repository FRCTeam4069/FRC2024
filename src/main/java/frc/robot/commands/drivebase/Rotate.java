package frc.robot.commands.drivebase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DrivebaseConstants.AlignConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class Rotate extends Command {
    private SwerveDrivetrain drive;
    private PIDController pid;
    private double radians;

    public Rotate(SwerveDrivetrain drive, double radians) {
        this.drive = drive;
        pid = new PIDController(AlignConstants.kP, AlignConstants.kI, AlignConstants.kD);
        pid.setTolerance(AlignConstants.positionTolerance, AlignConstants.velocityTolerance);
        pid.enableContinuousInput(-Math.PI, Math.PI);
        this.radians = radians;
        
        addRequirements(drive);
    }

    @Override
    public void end(boolean interrupted) {
        drive.fieldOrientedDrive(new ChassisSpeeds());
        SmartDashboard.putBoolean("align", false);
    }
    @Override
    public void execute() {
        var speed = -1*pid.calculate(drive.getNormalizedRads(), radians);
        speed = speed + Math.abs(AlignConstants.kS)*Math.signum(speed);
        drive.fieldOrientedDrive(new ChassisSpeeds(0, 0, speed));
        SmartDashboard.putBoolean("align", true);
        SmartDashboard.putNumber("align target", radians);
    }
    @Override
    public void initialize() {
        super.initialize();
    }
    @Override
    public boolean isFinished() {
        
        return pid.atSetpoint();
    }


    
}
