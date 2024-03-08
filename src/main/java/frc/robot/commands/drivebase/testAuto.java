package frc.robot.commands.drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoCustomAngle;
import frc.robot.commands.AutoLowerIntake;
import frc.robot.commands.AutoSetIntakeState;
import frc.robot.commands.AutoShooterCommand;
import frc.robot.commands.AutoStopShooter;
import frc.robot.commands.DefualtIndexerCommand;
import frc.robot.commands.DisableIndexCommand;
import frc.robot.commands.DisableSubsystems;
import frc.robot.commands.IndexShooter;
import frc.robot.commands.IndexWithSensorCommand;
import frc.robot.commands.IndexWithTime;
import frc.robot.commands.ShooterPositions;
import frc.robot.subsystems.IndexerController;
import frc.robot.subsystems.IntakeController;
import frc.robot.subsystems.ShooterController;
import frc.robot.subsystems.ShooterRotationController;
import frc.robot.subsystems.IntakeController.positions;
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
            new SequentialCommandGroup(
                new InstantCommand(() -> drive.setPose(new Pose2d(1.3, 5.55, Rotation2d.fromDegrees(180.0)))),
                new ParallelCommandGroup(
                    new AutoCustomAngle(rot, shooter, ShooterPositions.WALL_AREA),
                    new SequentialCommandGroup(
                        new WaitCommand(1),
                        new IndexWithTime(index, 1.0),
                        new WaitCommand(0.5),
                        new DisableIndexCommand(index)
                    ),
                    new InstantCommand(() -> SmartDashboard.putString("auto location", "first parallel"))
                ),
                new AutoShooterCommand(rot, shooter, index, ShooterPositions.SAFE_ZONE),

                new ParallelCommandGroup(
                    new AutoLowerIntake(intake),
                    
                    
                    new InstantCommand(() -> SmartDashboard.putString("auto location", "second parallel"))

                    //new AutoLowerIntake(i)
                ),

                

                //new AutoStopShooter(shooter),
                
                new InstantCommand(() -> SmartDashboard.putString("auto location", "start path")),

                new ParallelCommandGroup(
                    new AutoSetIntakeState(intake, frc.robot.commands.AutoSetIntakeState.State.ON),
                    new IndexWithTime(index, 1.4)
                ),
                
                new FollowPath(drive, "one"),

                

                new InstantCommand(() -> SmartDashboard.putString("auto location", "end path")),

                
                //new IndexWithTime(index, 1.0),
                //new AutoStopShooter(shooter, rot),

                new InstantCommand(() -> SmartDashboard.putString("auto location", "In hell")),

                new WaitCommand(1),
                new DisableSubsystems(rot, shooter, index, i)

                //new WaitCommand(5)

                // new ParallelCommandGroup(
                //     new AutoShooterCommand(rot, shooter, index, ShooterPositions.WALL_AREA),
                //     new SequentialCommandGroup(
                //         new WaitCommand(1),
                //         new IndexShooter(index)
                //     )
                
               
                //new AutoShooterCommand(rot, shooter, index, null)

                
                    
                //new AutoShooterCommand(rot, shooter, index)

                //new AutoStopShooter(shooter)
                
                //new WaitCommand(3)
                                


            )
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
