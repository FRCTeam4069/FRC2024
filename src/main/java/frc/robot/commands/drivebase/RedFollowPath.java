package frc.robot.commands.drivebase;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.DrivebaseConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class RedFollowPath extends SequentialCommandGroup {
    public RedFollowPath(SwerveDrivetrain drive, String pathName) {
        addRequirements(drive);
        addCommands(
            new FollowPathHolonomic(
                PathPlannerPath.fromPathFile(pathName), 
                drive::getPose, 
                drive::getRobotRelativeSpeeds, 
                drive::drive, 
                new HolonomicPathFollowerConfig(
                    DrivebaseConstants.translation, //translation 0.6
                    DrivebaseConstants.rotation, //rotation
                    DrivebaseConstants.maxVelocity, 
                    DrivebaseConstants.drivebaseRadius, 
                    new ReplanningConfig()
                    ),
                () -> true,
                drive));
    }

    public RedFollowPath(SwerveDrivetrain drive, String pathName, double timeout) {
        addRequirements(drive);
        addCommands(
            new FollowPathHolonomic(
                PathPlannerPath.fromPathFile(pathName), 
                drive::getPose, 
                drive::getRobotRelativeSpeeds, 
                drive::drive, 
                new HolonomicPathFollowerConfig(
                    DrivebaseConstants.translation, //translation 0.6
                    DrivebaseConstants.rotation, //rotation
                    DrivebaseConstants.maxVelocity, 
                    DrivebaseConstants.drivebaseRadius, 
                    new ReplanningConfig()
                    ),
                timeout,
                () -> true,
                drive));
    }

    public RedFollowPath(SwerveDrivetrain drive, String pathName, PIDConstants translation, double timeout) {
        addRequirements(drive);
        addCommands(
            new FollowPathHolonomic(
                PathPlannerPath.fromPathFile(pathName), 
                drive::getPose, 
                drive::getRobotRelativeSpeeds, 
                drive::drive, 
                new HolonomicPathFollowerConfig(
                    translation, //translation 0.6
                    DrivebaseConstants.rotation, //rotation
                    DrivebaseConstants.maxVelocity, 
                    DrivebaseConstants.drivebaseRadius, 
                    new ReplanningConfig()
                    ),
                timeout,
                () -> true,
                drive));
    }

    public RedFollowPath(SwerveDrivetrain drive, String pathName, PIDConstants translation, PIDConstants rotation, double timeout) {
        addRequirements(drive);
        addCommands(
            new FollowPathHolonomic(
                PathPlannerPath.fromPathFile(pathName), 
                drive::getPose, 
                drive::getRobotRelativeSpeeds, 
                drive::drive, 
                new HolonomicPathFollowerConfig(
                    translation, //translation 0.6
                    rotation, //rotation
                    DrivebaseConstants.maxVelocity, 
                    DrivebaseConstants.drivebaseRadius, 
                    new ReplanningConfig()
                    ),
                timeout,
                () -> true,
                drive));
    }
    
}
