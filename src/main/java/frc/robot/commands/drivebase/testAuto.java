package frc.robot.commands.drivebase;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoLowerIntake;
import frc.robot.commands.AutoSetIntakeState;
import frc.robot.constants.DrivebaseConstants;
import frc.robot.subsystems.IntakeController;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class testAuto extends SequentialCommandGroup {
    private SwerveDrivetrain drive;
    private IntakeController intake;
    public testAuto(SwerveDrivetrain drive, IntakeController i) {
        this.drive = drive;
        intake = i;
        addRequirements(drive, intake);

        drive.setPose(new Pose2d(1.3, 5.55, Rotation2d.fromDegrees(180.0)));
        addCommands(
            
            new AutoLowerIntake(intake),
            new FollowPath(drive, "go to first ring"),
            new AutoSetIntakeState(intake, frc.robot.commands.AutoSetIntakeState.State.ON)
            
            //new FollowPath(drive, "go to second ring")
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
