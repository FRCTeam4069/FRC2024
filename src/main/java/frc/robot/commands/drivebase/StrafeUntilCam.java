package frc.robot.commands.drivebase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DrivebaseConstants;
import frc.robot.subsystems.IntakeController.positions;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class StrafeUntilCam extends Command {
    private SwerveDrivetrain drive;
    private double direction = 1;
    private DoubleSupplier angle;
    private BooleanSupplier hasTarget;
    private PIDController pid;
    private double power = 0;
    private boolean hadTargets;
    private double angleBuffer = 0.0;

    public StrafeUntilCam(SwerveDrivetrain drive, DoubleSupplier cameraAngle, double direction, BooleanSupplier hasTarget) {
        this.drive = drive;
        this.direction = direction;
        this.angle = cameraAngle;
        this.hasTarget = hasTarget;

        addRequirements(drive);

    }
    
    @Override
    public void initialize() {
        pid = new PIDController(0.01, 0.0, 0.0);
        pid.setTolerance(0.0001);

    }

    @Override
    public void execute() {
        angleBuffer = angle.getAsDouble();
        if (hasTarget.getAsBoolean()) {
            power = pid.calculate(angleBuffer, 0.0);
            power += (0.0001)*Math.signum(power);
        } else if (hadTargets){
            power = pid.calculate(angleBuffer, 0.0);
            power += (0.0001)*Math.signum(power);
        } else {
            power = direction;
        }

        MathUtil.clamp(power, -0.5, 0.5);
        drive.fieldOrientedDrive(new ChassisSpeeds(0, power*DrivebaseConstants.maxVelocity, 0));
        SmartDashboard.putNumber("strafe power", power);
        SmartDashboard.putBoolean("straf", true);
    }


    @Override
    public boolean isFinished() {
        SmartDashboard.putBoolean("straf", false);
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(new ChassisSpeeds());
    }
    
}
