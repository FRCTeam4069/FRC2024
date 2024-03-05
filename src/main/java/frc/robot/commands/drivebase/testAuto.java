package frc.robot.commands.drivebase;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.DrivebaseConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class testAuto extends SequentialCommandGroup {
    private SwerveDrivetrain drive;
    public testAuto(SwerveDrivetrain drive) {
        this.drive = drive;
        addRequirements(drive);

        addCommands(
        new FollowPathHolonomic(
            PathPlannerPath.fromPathFile("go to first ring"), 
            drive::getPose, 
            drive::getRobotRelativeSpeeds, 
            drive::drive, 
            new HolonomicPathFollowerConfig(
                DrivebaseConstants.maxVelocity, 
                DrivebaseConstants.drivebaseRadius, 
                new ReplanningConfig()),
            () -> false,
            drive)
        );


    }

}
