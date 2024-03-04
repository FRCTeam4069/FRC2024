package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DrivebaseConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class FieldCentricDrive extends Command {
    private final SwerveDrivetrain drive;
    private final DoubleSupplier forwardSpeed;
    private final DoubleSupplier strafeSpeed;
    private final DoubleSupplier turnSpeed;
    private final BooleanSupplier halfSpeed;
    public FieldCentricDrive(SwerveDrivetrain drive, DoubleSupplier forwardSpeed, DoubleSupplier strafeSpeed, DoubleSupplier turnSpeed, BooleanSupplier halfSpeed) {
        this.drive = drive;
        this.turnSpeed = turnSpeed;
        this.forwardSpeed = forwardSpeed;
        this.strafeSpeed = strafeSpeed;
        this.halfSpeed = halfSpeed;
        addRequirements(drive);
    }
    @Override
    public void initialize() {
        
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

        SmartDashboard.putString("teleop running", "yes");

        drive.fieldOrientedDrive(new ChassisSpeeds(
                (Math.pow(forwardSpeed.getAsDouble(), 3)*speedMultiplier * DrivebaseConstants.maxVelocity),
                (Math.pow(strafeSpeed.getAsDouble(), 3)*speedMultiplier * DrivebaseConstants.maxVelocity),
                (Math.pow(turnSpeed.getAsDouble(), 3)*speedMultiplier * DrivebaseConstants.maxAngularVelocity)));

    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
    
}
