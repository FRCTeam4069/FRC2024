package frc.robot.commands.drivebase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DrivebaseConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class CameraPIDCommand extends Command {
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;
    private SwerveDrivetrain drive;
    private Pose2d setpoint;

    public CameraPIDCommand(SwerveDrivetrain drive, Pose2d setpoint) {
        this.drive = drive;
        this.setpoint = setpoint;
        xController = new PIDController(6.00, 0.0, 0.05); //translation
        yController = new PIDController(6.00, 0.0, 0.05); //translation
        rotationController = new PIDController(6.0, 0.0, 0.1); //rotation
        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        xController.setTolerance(0.005, 0.01);
        yController.setTolerance(0.005, 0.01);
        rotationController.setTolerance(Units.degreesToRadians(0.5), Units.degreesToRadians(0.5));

    }

    @Override
    public void initialize() {

    }


    @Override
    public void execute() {
        var currentPose = drive.getCameraPose();
        double xFeedback =
            this.xController.calculate(currentPose.getX(), setpoint.getX());
        double yFeedback =
            this.yController.calculate(currentPose.getY(), setpoint.getY());

        double rotationFeedback =
            rotationController.calculate(
                currentPose.getRotation().getRadians(),
                setpoint.getRotation().getRadians());

        drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
            xFeedback, yFeedback, -1*(rotationFeedback), currentPose.getRotation()));
            
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint() && yController.atSetpoint() && rotationController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }

    

    
}
