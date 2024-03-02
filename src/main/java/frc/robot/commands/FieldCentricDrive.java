package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

@Deprecated
public class FieldCentricDrive extends Command {
    private final SwerveSubsystem drive;
    private final DoubleSupplier forwardSpeed;
    private final DoubleSupplier strafeSpeed;
    private final DoubleSupplier turnSpeed;
    public FieldCentricDrive(SwerveSubsystem drive, DoubleSupplier forwardSpeed, DoubleSupplier strafeSpeed, DoubleSupplier turnSpeed) {
        this.drive = drive;
        this.turnSpeed = turnSpeed;
        this.forwardSpeed = forwardSpeed;
        this.strafeSpeed = strafeSpeed;
        addRequirements(drive);
    }
    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        drive.drive(new ChassisSpeeds(forwardSpeed.getAsDouble(), strafeSpeed.getAsDouble(), turnSpeed.getAsDouble()));
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
    
}
