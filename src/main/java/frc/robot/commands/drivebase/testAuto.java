package frc.robot.commands.drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoLowerIntake;
import frc.robot.commands.AutoSetIntakeState;
import frc.robot.commands.AutoShooterCommand;
import frc.robot.subsystems.IndexerController;
import frc.robot.subsystems.IntakeController;
import frc.robot.subsystems.ShooterController;
import frc.robot.subsystems.ShooterRotationController;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class testAuto extends SequentialCommandGroup {
    private SwerveDrivetrain drive;
    private IntakeController intake;
    private IndexerController indexer;
    private ShooterController shooter;
    private ShooterRotationController rotShoot;
    public testAuto(SwerveDrivetrain drive, IntakeController i, IndexerController index, ShooterController shooter, ShooterRotationController rot) {
        this.drive = drive;
        intake = i;
        indexer = index;
        this.shooter = shooter;
        this.rotShoot = rot;
        addRequirements(drive, intake, shooter, rot, index);

        drive.setPose(new Pose2d(1.3, 5.55, Rotation2d.fromDegrees(180.0)));
        addCommands(
            
            new AutoLowerIntake(intake),
            new FollowPath(drive, "one"),
            new AutoSetIntakeState(intake, frc.robot.commands.AutoSetIntakeState.State.ON)
            //new AutoShooterCommand(rot, shooter, index)
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
