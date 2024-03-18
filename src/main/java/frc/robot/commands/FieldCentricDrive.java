package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.cscore.CameraServerJNI.TelemetryKind;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DrivebaseConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.DrivebaseConstants.AlignConstants;
import frc.robot.constants.DrivebaseConstants.AutoAlignConstants;
import frc.robot.subsystems.Limelight.CameraIsAsCameraDoes;
import frc.robot.subsystems.Limelight.PoseEstimatorSubsystem;
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
    private DoubleSupplier angle;
    private double lastTimestamp;
    private double angleBuffer;
    private ShuffleboardTab tab;
    private BooleanSupplier fieldCentric;
    private SlewRateLimiter xSlewRateLimiter = new SlewRateLimiter(20.0);
    private SlewRateLimiter ySlewRateLimiter = new SlewRateLimiter(20.0);
    private SlewRateLimiter wSlewRateLimiter = new SlewRateLimiter(20.0);
    public FieldCentricDrive(SwerveDrivetrain drive, DoubleSupplier forwardSpeed, DoubleSupplier strafeSpeed, DoubleSupplier turnSpeed, BooleanSupplier halfSpeed, BooleanSupplier autoAlign, DoubleSupplier angleToAlign, BooleanSupplier fieldCentric) {
        this.drive = drive;
        this.turnSpeed = turnSpeed;
        this.forwardSpeed = forwardSpeed;
        this.strafeSpeed = strafeSpeed;
        this.halfSpeed = halfSpeed;
        this.autoAlign = autoAlign;
        this.angle = angleToAlign;
        this.fieldCentric = fieldCentric;


        addRequirements(drive);
    }
    @Override
    public void initialize() {

        headingPID = new PIDController(AutoAlignConstants.kP, AutoAlignConstants.kI, AutoAlignConstants.kD);
        headingPID.setTolerance(AutoAlignConstants.positionTolerance, AlignConstants.velocityTolerance);
        headingPID.enableContinuousInput(-Math.PI, Math.PI);

        headingFilter = new MedianFilter(10);
        
        lastTimestamp = Timer.getFPGATimestamp();

        tab = Shuffleboard.getTab("rotation");
    }

    @Override
    public void execute() {
        var currentTime = Timer.getFPGATimestamp();
        var speedMultiplier = 1.0;
        // if (Math.abs(forwardSpeed.getAsDouble()) < 0.01 && Math.abs(strafeSpeed.getAsDouble()) < 0.01 && Math.abs(turnSpeed.getAsDouble()) < 0.01) {
            // speedMultiplier = 0;
        if (halfSpeed.getAsBoolean()) {
            speedMultiplier = 0.1;
        } else {
            speedMultiplier = 1.0;
        }

        if (currentTime-lastTimestamp > 1.0) {
            angleBuffer = angle.getAsDouble() + drive.getRadians();
            lastTimestamp = Timer.getFPGATimestamp();
        }

        //var targetAngle = headingFilter.calculate(cam.getTargetRotation().getY());
        SmartDashboard.putNumber("camera target angle", angle.getAsDouble());
        SmartDashboard.putNumber("camera target angle buffer", angleBuffer);
        SmartDashboard.putBoolean("isAligned", MathUtil.isNear(0.0, angle.getAsDouble(), Units.degreesToRadians(3)));
        

        var outputSpeeds = new ChassisSpeeds();
        if (!autoAlign.getAsBoolean()) {
            drive.setInputLimit(true);
            outputSpeeds = new ChassisSpeeds(
                xSlewRateLimiter.calculate(Math.pow(MathUtil.applyDeadband(forwardSpeed.getAsDouble(), 0.15), 3)*speedMultiplier * DrivebaseConstants.maxVelocity),
                ySlewRateLimiter.calculate(Math.pow(MathUtil.applyDeadband(strafeSpeed.getAsDouble(), 0.15), 3)*speedMultiplier * DrivebaseConstants.maxVelocity),
                wSlewRateLimiter.calculate(Math.pow(MathUtil.applyDeadband(turnSpeed.getAsDouble(), 0.15), 3)*speedMultiplier * DrivebaseConstants.maxVelocity));
        } else {
            // var power = headingPID.calculate(drive.getNormalizedRads(), 0.0);
            // var voltage = (12 - RobotController.getBatteryVoltage()) * AutoAlignConstants.kV * Math.signum(power);
            // power += Math.abs(AutoAlignConstants.kS)*Math.signum(power) + voltage*Math.signum(power);
            // var xSpeed = forwardSpeed.getAsDouble();
            // var ySpeed = strafeSpeed.getAsDouble();

            // if (xSpeed > 0.2 || ySpeed > 0.2) {
            //     power = power * 180;
            // }



            //drive.setInputLimit(false);
            // var translation = cam.getTargetTranslation(7);
            // var targetAngle = Math.atan2(translation.getY(), translation.getX());
            //var power = headingPID.calculate(drive.getDegrees(), angle.getAsDouble());

            // var power = headingPID.calculate(drive.getRadians(), angleBuffer);
            // power = power + DrivebaseConstants.AutoAlignConstants.kS*Math.signum(power);
            
            // if (power > AutoAlignConstants.powerLimit || power < -AutoAlignConstants.powerLimit) {
            //     power = AutoAlignConstants.powerLimit*Math.signum(power);
            // }
            // SmartDashboard.putNumber("align power", power);


            // if (Math.abs(drive.getRadians()) < Units.degreesToRadians(3)) power = 0;
            // if (Math.abs(power)>1) power=Math.signum(power);

            var robotPose = drive.getPose();
            var targetPose = FieldConstants.poseBlueSpeaker;

            var relativePose = robotPose.minus(targetPose);
            var desiredAngle = Math.atan2(relativePose.getY(), relativePose.getX());

            var power = headingPID.calculate(drive.getNormalizedRads(), SwerveDrivetrain.normalizeRadians(desiredAngle+Math.PI));

            
            outputSpeeds = new ChassisSpeeds(
                (Math.pow(forwardSpeed.getAsDouble(), 3)*speedMultiplier * DrivebaseConstants.maxVelocity),
                (Math.pow(strafeSpeed.getAsDouble(), 3)*speedMultiplier * DrivebaseConstants.maxVelocity),
                (power)*DrivebaseConstants.maxAngularVelocity);

        }

        if (fieldCentric.getAsBoolean()) {
            drive.fieldOrientedDrive(outputSpeeds);

        } else {
            drive.drive(outputSpeeds);
        }



        SmartDashboard.putString("teleop running", "yes");


    }
    
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.setInputLimit(true);
        SmartDashboard.putBoolean("isAligned", false);
    }
    
}
