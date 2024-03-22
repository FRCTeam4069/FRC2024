package frc.robot.commands.drivebase.test;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoCustomAngle;
import frc.robot.commands.AutoLowerIntake;
import frc.robot.commands.AutoSetIntakeState;
import frc.robot.commands.AutoShooterCommand;
import frc.robot.commands.BetterIndexerCommandWithStop;
import frc.robot.commands.CustomShooterCommand;
import frc.robot.commands.DisableIndexCommand;
import frc.robot.commands.DisableSubsystems;
import frc.robot.commands.IndexWithTime;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RotateShooterCommand;
import frc.robot.commands.ShooterPositions;
import frc.robot.commands.drivebase.RedFollowPath;
import frc.robot.commands.drivebase.Rotate;
import frc.robot.subsystems.IndexerController;
import frc.robot.subsystems.IntakeController;
import frc.robot.subsystems.ShooterController;
import frc.robot.subsystems.ShooterRotationController;
import frc.robot.subsystems.IntakeController.positions;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class RedTest extends SequentialCommandGroup {
    private SwerveDrivetrain drive;
    private IntakeController intake;
    private IndexerController indexer;
    private ShooterController shooter;
    private ShooterRotationController rotShoot;
    public RedTest(SwerveDrivetrain drive, IntakeController i, IndexerController index, ShooterController shooter, ShooterRotationController rot) {
        this.drive = drive;
        intake = i;
        indexer = index;
        this.shooter = shooter;
        this.rotShoot = rot;
        addRequirements(drive, intake, shooter, rot, index);

        addCommands(
            new SequentialCommandGroup(
                new InstantCommand(() -> drive.setPose(new Pose2d(1.45, 6.50, Rotation2d.fromDegrees(0.0)))),
                new InstantCommand(() -> drive.setPose(new Pose2d(1.45, 6.50, Rotation2d.fromDegrees(0.0)))),

                new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new Rotate(drive, Units.degreesToRadians(34)),
                        new CustomShooterCommand(rot, shooter, 58, 40)
                        //new AutoCustomAngle(rot, shooter, ShooterPositions.WALL_AREA)
                    ),
                    new IndexWithTime(index, 1.0),
                    new WaitCommand(0.2)
                ),

                new ParallelCommandGroup(
                    //new AutoSetIntakeState(intake, frc.robot.commands.AutoSetIntakeState.State.ON),
                    new AutoLowerIntake(intake),
                    new InstantCommand(() -> i.setIntakeSpeed(-0.80)),
                    new SequentialCommandGroup(
                        // new WaitCommand(0.5),
                        new ParallelCommandGroup(
                            //new BetterIndexerCommandWithStop(index).withTimeout(10),
                            //new InstantCommand(() -> i.driveFeed()),
                            new RotateShooterCommand(rot, 70)
                        )
                        // new AutoShooterCommand(rot, shooter, index, ShooterPositions.SAFE_ZONE)
                    ),
                    new SequentialCommandGroup(
                        new Rotate(drive, 0),
                        new InstantCommand(() -> drive.stopModules())
                    )
                ),

                new ParallelCommandGroup(
                    new BetterIndexerCommandWithStop(index).withTimeout(10),
                    new RotateShooterCommand(rot, 70),
                    new SequentialCommandGroup(
                        new RedFollowPath(drive, "2056 p1"),
                        new InstantCommand(() -> drive.stopModules())
                    )
                ),

                // new InstantCommand(() -> SmartDashboard.putString("auto location", "WOOO YA BABA THAT'S WHAT I'VE BEEN WAITING FOR THAT'S WHAT IT'S ALL ABOUT")),

                new ParallelCommandGroup(
                    new Rotate(drive, Units.degreesToRadians(31)),
                    // new AutoShooterCommand(rot, shooter, index, ShooterPositions.SAFE_ZONE),
                    // new AutoCustomAngle(rot, shooter, ShooterPositions.WALL_AREA),
                    new CustomShooterCommand(rot, shooter, 60, 53, 0.80),
                    new InstantCommand(() -> i.stopFeed())
                ),

            //      new InstantCommand(() -> drive.stopModules()),
                new ParallelCommandGroup(
                    new InstantCommand(() -> i.stopFeed()),
                    new IndexWithTime(index, 1)
                ),
                new WaitCommand(0.2),

                new ParallelCommandGroup(
                    new Rotate(drive, Units.degreesToRadians(0)),
                    new InstantCommand(() -> i.stopFeed())
                ),

                new ParallelCommandGroup(
                    //new AutoSetIntakeState(intake, frc.robot.commands.AutoSetIntakeState.State.ON),
                    new SequentialCommandGroup(
                        new InstantCommand(() -> i.stopFeed()),
                        new InstantCommand(() -> index.stop()),
                        new WaitCommand(2.0),
                        new ParallelDeadlineGroup(
                            new BetterIndexerCommandWithStop(index).withTimeout(5),
                            new ParallelCommandGroup(
                                new RotateShooterCommand(rot, 70),
                                new InstantCommand(() -> i.setIntakeSpeed(-0.80))
                            )
                        )
                    ),
                    new SequentialCommandGroup(
                        new RedFollowPath(drive, "2056 p2", new PIDConstants(1.5), 0.5),
                        new InstantCommand(() -> drive.stopModules())
                    )
                ),

                new ParallelCommandGroup(
                    new SequentialCommandGroup(
                        new RedFollowPath(drive, "2056 p3", new PIDConstants(0.8), 0.5),
                        new InstantCommand(() -> drive.stopModules())
                    ),
                    //new AutoShooterCommand(rot, shooter, index, ShooterPositions.SAFE_ZONE),
                    new CustomShooterCommand(rot, shooter, 80, 59.5, 0.5, 0.5),
                    new InstantCommand(() -> i.stopFeed()),
                    new IntakeCommand(i, positions.UPPER)
                ),
                new WaitCommand(0.2),
                new IndexWithTime(index, 1),
                new WaitCommand(1),


            //     new ParallelCommandGroup(
            //         new SequentialCommandGroup(
            //             new FollowPath(drive, "four ring p3"),
            //             new InstantCommand(() -> drive.stopModules())
            //         ),
            //         new AutoShooterCommand(rot, shooter, index, ShooterPositions.SAFE_ZONE),
            //         new InstantCommand(() -> i.stopFeed())
            //     ),
            //     //new WaitCommand(1),
            //     new IndexWithTime(index, 1),

            //     new ParallelCommandGroup(
            //         //new AutoSetIntakeState(intake, frc.robot.commands.AutoSetIntakeState.State.ON),
            //         new InstantCommand(() -> i.setIntakeSpeed(-0.65)),
            //         new SequentialCommandGroup(
            //             new WaitCommand(0.5),
            //             new ParallelDeadlineGroup(
            //                 new BetterIndexerCommandWithStop(index).withTimeout(3),
            //                 new RotateShooterCommand(rot, 70)
            //             )
            //         ),
            //         new SequentialCommandGroup(
            //             new FollowPath(drive, "four ring p4"),
            //             new InstantCommand(() -> drive.stopModules())
            //         )

            //     ),

            //     new ParallelCommandGroup(
            //         new SequentialCommandGroup(
            //             new FollowPath(drive, "four ring p5"),
            //             new InstantCommand(() -> drive.stopModules())
            //         ),
            //         new AutoShooterCommand(rot, shooter, index, ShooterPositions.SAFE_ZONE),
            //         new InstantCommand(() -> i.stopFeed())
            //     ),
            //     new WaitCommand(0.25),
            //     new IndexWithTime(index, 2),
            //     new WaitCommand(3),

                new DisableSubsystems(rot, shooter, index, i)

            )
        );


    }

    

}
