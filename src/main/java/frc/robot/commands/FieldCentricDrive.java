package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
    // private DoubleSupplier angle;
    private Supplier<Transform2d> speakerTransform;
    private double lastTimestamp;
    private double angleBuffer;
    private BooleanSupplier speakerAlign;
    private BooleanSupplier spinSupplier;
    private Supplier<Rotation2d> rotationSupplier;
    private SlewRateLimiter xSlewRateLimiter = new SlewRateLimiter(20.0);
    private SlewRateLimiter ySlewRateLimiter = new SlewRateLimiter(20.0);
    private SlewRateLimiter wSlewRateLimiter = new SlewRateLimiter(20.0);
    public FieldCentricDrive(SwerveDrivetrain drive, DoubleSupplier forwardSpeed, DoubleSupplier strafeSpeed, DoubleSupplier turnSpeed, BooleanSupplier halfSpeed, BooleanSupplier autoAlign, Supplier<Transform2d> speakerTransform, BooleanSupplier speakerAlign, BooleanSupplier spin, Supplier<Rotation2d> rotationSupplier) {
        this.drive = drive;
        this.turnSpeed = turnSpeed;
        this.forwardSpeed = forwardSpeed;
        this.strafeSpeed = strafeSpeed;
        this.halfSpeed = halfSpeed;
        this.autoAlign = autoAlign;
        this.speakerTransform = speakerTransform;
        this.speakerAlign = speakerAlign;
        this.spinSupplier = spin;
        this.rotationSupplier = rotationSupplier;


        addRequirements(drive);
    }
    @Override
    public void initialize() {

        headingPID = new PIDController(AutoAlignConstants.kP, AutoAlignConstants.kI, AutoAlignConstants.kD);
        headingPID.setTolerance(AutoAlignConstants.positionTolerance, AlignConstants.velocityTolerance);
        headingPID.enableContinuousInput(-Math.PI, Math.PI);

        headingFilter = new MedianFilter(10);
        
        lastTimestamp = Timer.getFPGATimestamp();

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

        //var targetAngle = headingFilter.calculate(cam.getTargetRotation().getY());
        // SmartDashboard.putNumber("camera target angle", angle.getAsDouble());
        // SmartDashboard.putNumber("camera target angle buffer", angleBuffer);
        // SmartDashboard.putBoolean("isAligned", MathUtil.isNear(0.0, angle.getAsDouble(), Units.degreesToRadians(3)));

        var turnPower = wSlewRateLimiter.calculate(Math.pow(MathUtil.applyDeadband(turnSpeed.getAsDouble(), 0.15), 3)*speedMultiplier * DrivebaseConstants.maxVelocity);
        var speaker = speakerAlign.getAsBoolean();
        var straight = autoAlign.getAsBoolean();
        var spin = spinSupplier.getAsBoolean();
        if (spin && (speaker || straight) || (speaker && straight)) {
            ;
        } else if (speakerAlign.getAsBoolean()) {

            var relativePose = speakerTransform.get();
            var desiredAngle = Math.atan2(relativePose.getY(), relativePose.getX());

            turnPower = -1*headingPID.calculate(SwerveDrivetrain.normalizeRadians(rotationSupplier.get().getRadians()), SwerveDrivetrain.normalizeRadians(desiredAngle));
            
            // outputSpeeds = new ChassisSpeeds(
            //     (Math.pow(forwardSpeed.getAsDouble(), 3)*speedMultiplier * DrivebaseConstants.maxVelocity),
            //     (Math.pow(strafeSpeed.getAsDouble(), 3)*speedMultiplier * DrivebaseConstants.maxVelocity),
            //     (power)*DrivebaseConstants.maxAngularVelocity);

        } else if (autoAlign.getAsBoolean()) {
            // SmartDashboard.putNumber("normalized rads", drive.getNormalizedRads());
            // SmartDashboard.putNumber("angle error", drive.getNormalizedRads()-Math.PI);
            turnPower = -1*headingPID.calculate(drive.getNormalizedRads(), Units.degreesToRadians(0.0));
            // var voltage = (12 - RobotController.getBatteryVoltage()) * AutoAlignConstants.kV * Math.signum(power);
            // power += Math.abs(AutoAlignConstants.kS)*Math.signum(power) + voltage*Math.signum(power);

        } else if (spinSupplier.getAsBoolean()) {
            turnPower = 100;
        }

        drive.setInputLimit(true);
        var outputSpeeds = new ChassisSpeeds(
            xSlewRateLimiter.calculate(Math.pow(MathUtil.applyDeadband(forwardSpeed.getAsDouble(), 0.15), 3)*speedMultiplier * DrivebaseConstants.maxVelocity),
            ySlewRateLimiter.calculate(Math.pow(MathUtil.applyDeadband(strafeSpeed.getAsDouble(), 0.15), 3)*speedMultiplier * DrivebaseConstants.maxVelocity),
            turnPower);

        drive.fieldOrientedDrive(outputSpeeds);

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
