package frc.robot.commands.drivebase;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.DrivebaseConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class testAuto extends SequentialCommandGroup {
    private SwerveDrivetrain drive;
    public testAuto(SwerveDrivetrain drive) {
        this.drive = drive;
        addRequirements(drive);

        drive.setPose(new Pose2d(1.3, 5.55, Rotation2d.fromDegrees(180.0)));
        addCommands(
            new FollowPath(drive, "go to first ring"),
            new FollowPath(drive, "go to second ring")
            // new FollowPathHolonomic(
            //     PathPlannerPath.fromPathFile("go to first ring"), 
            //     drive::getPose, 
            //     drive::getRobotRelativeSpeeds, 
            //     drive::drive, 
            //     new HolonomicPathFollowerConfig(
            //         new PIDConstants(0.4, 0.0, 0.0), //translation
            //         new PIDConstants(0.0, 0.0, 0.0), //rotation
            //         DrivebaseConstants.maxVelocity-3.0, 
            //         DrivebaseConstants.drivebaseRadius, 
            //         new ReplanningConfig()
            //         ),
            //     () -> false,
            //     drive)
        );


    }

}
