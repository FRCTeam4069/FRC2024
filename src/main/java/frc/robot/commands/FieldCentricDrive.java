package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.cscore.CameraServerJNI.TelemetryKind;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DrivebaseConstants;
import frc.robot.constants.DrivebaseConstants.AutoAlignConstants;
import frc.robot.subsystems.Limelight.CameraIsAsCameraDoes;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class FieldCentricDrive extends Command {
    private final SwerveDrivetrain drive;
    private final DoubleSupplier forwardSpeed;
    private final DoubleSupplier strafeSpeed;
    private final DoubleSupplier turnSpeed;
    private final BooleanSupplier halfSpeed;
    private final BooleanSupplier autoAlign;
    private PIDController headingPID;
    private MedianFilter headingFilter;
    private final CameraIsAsCameraDoes cam;
    public FieldCentricDrive(SwerveDrivetrain drive, CameraIsAsCameraDoes cam, DoubleSupplier forwardSpeed, DoubleSupplier strafeSpeed, DoubleSupplier turnSpeed, BooleanSupplier halfSpeed, BooleanSupplier autoAlign) {
        this.drive = drive;
        this.turnSpeed = turnSpeed;
        this.forwardSpeed = forwardSpeed;
        this.strafeSpeed = strafeSpeed;
        this.halfSpeed = halfSpeed;
        this.autoAlign = autoAlign;
        this.cam = cam;
        addRequirements(drive);
    }
    @Override
    public void initialize() {
        headingPID = new PIDController(AutoAlignConstants.kP, AutoAlignConstants.kI, AutoAlignConstants.kD);
        headingPID.setTolerance(AutoAlignConstants.positionTolerance, AutoAlignConstants.velocityTolerance);
        headingPID.enableContinuousInput(-Math.PI, Math.PI);

        headingFilter = new MedianFilter(10);
        
    }

    @Override
    public void execute() {
        var speedMultiplier = 1.0;
        // if (Math.abs(forwardSpeed.getAsDouble()) < 0.01 && Math.abs(strafeSpeed.getAsDouble()) < 0.01 && Math.abs(turnSpeed.getAsDouble()) < 0.01) {
            // speedMultiplier = 0;
        if (halfSpeed.getAsBoolean()) {
            speedMultiplier = 0.1;
        } else {
            speedMultiplier = 1.0;
        }

        var targetAngle = headingFilter.calculate(cam.getTargetRotation().getY());
        SmartDashboard.putNumber("camera target angle", targetAngle);

        if (!autoAlign.getAsBoolean()) {
            drive.fieldOrientedDrive(new ChassisSpeeds(
                (Math.pow(forwardSpeed.getAsDouble(), 3)*speedMultiplier * DrivebaseConstants.maxVelocity),
                (Math.pow(strafeSpeed.getAsDouble(), 3)*speedMultiplier * DrivebaseConstants.maxVelocity),
                (Math.pow(turnSpeed.getAsDouble(), 3)*speedMultiplier * DrivebaseConstants.maxAngularVelocity)));
        } else {
            //var translation = cam.getTargetTranslation(7);
            //var targetAngle = Math.atan2(translation.getY(), translation.getX());
            
            drive.fieldOrientedDrive(new ChassisSpeeds(
                (Math.pow(forwardSpeed.getAsDouble(), 3)*speedMultiplier * DrivebaseConstants.maxVelocity),
                (Math.pow(strafeSpeed.getAsDouble(), 3)*speedMultiplier * DrivebaseConstants.maxVelocity),
                (-headingPID.calculate(drive.getNormalizedRads(), -1*targetAngle))));

        }

        SmartDashboard.putString("teleop running", "yes");


    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
    
}
