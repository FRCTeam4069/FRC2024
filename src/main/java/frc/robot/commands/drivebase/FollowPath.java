package frc.robot.commands.drivebase;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.DrivebaseConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class FollowPath extends SequentialCommandGroup {
    public FollowPath(SwerveDrivetrain drive, String pathName) {
        addRequirements(drive);
        addCommands(
            new FollowPathHolonomic(
                PathPlannerPath.fromPathFile(pathName), 
                drive::getPose, 
                drive::getRobotRelativeSpeeds, 
                drive::drive, 
                new HolonomicPathFollowerConfig(
                    new PIDConstants(0.9, 0.0, 0.0), //translation
                    new PIDConstants(1.0, 0.0, 0.0), //rotation
                    DrivebaseConstants.maxVelocity-3.0, 
                    DrivebaseConstants.drivebaseRadius, 
                    new ReplanningConfig()
                    ),
                () -> false,
                drive));
    }

    
}
