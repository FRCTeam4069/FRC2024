package frc.robot.commands.drivebase;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.commands.FasterIndexerCommandWithStop;
import frc.robot.commands.IndexWithTime;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RotateShooterCommand;
import frc.robot.commands.ShooterPositions;
import frc.robot.subsystems.IndexerController;
import frc.robot.subsystems.IntakeController;
import frc.robot.subsystems.ShooterController;
import frc.robot.subsystems.ShooterRotationController;
import frc.robot.subsystems.IntakeController.positions;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class FourCloseFaster extends SequentialCommandGroup {
    private SwerveDrivetrain drive;
    private IntakeController intake;
    private IndexerController indexer;
    private ShooterController shooter;
    private ShooterRotationController rotShoot;
    public FourCloseFaster(SwerveDrivetrain drive, IntakeController i, IndexerController index, ShooterController shooter, ShooterRotationController rot) {
        this.drive = drive;
        intake = i;
        indexer = index;
        this.shooter = shooter;
        this.rotShoot = rot;
        addRequirements(drive, intake, shooter, rot, index);

        addCommands(
            new SequentialCommandGroup(
                new InstantCommand(() -> drive.setPose(new Pose2d(1.30, 5.55, Rotation2d.fromDegrees(0.0)))),
                new InstantCommand(() -> drive.setPose(new Pose2d(1.30, 5.55, Rotation2d.fromDegrees(0.0)))),
                new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new AutoLowerIntake(intake),
                        // new AutoCustomAngle(rot, shooter, ShooterPositions.WALL_AREA),
                        new CustomShooterCommand(rot, shooter, 60, 35, 0.75, 3.0, 10.0),
                        // new SequentialCommandGroup(
                        //     new WaitCommand(1),
                        //     new IndexWithTime(index, 1.0),
                        //     new WaitCommand(0.5)
                        // ),
                        new InstantCommand(() -> SmartDashboard.putString("auto location", "first parallel"))
                    ),
                    new IndexWithTime(index, 1.0),
                    //new AutoShooterCommand(rot, shooter, index, ShooterPositions.SAFE_ZONE),

                    new InstantCommand(() -> SmartDashboard.putString("auto location", "start path"))

                ),

                new ParallelCommandGroup(
                    //new AutoSetIntakeState(intake, frc.robot.commands.AutoSetIntakeState.State.ON),
                    new AutoLowerIntake(intake),
                    new InstantCommand(() -> i.setIntakeSpeed(-0.80)),
                    new SequentialCommandGroup(
                        new WaitCommand(0.3),
                        new ParallelDeadlineGroup(
                            new BetterIndexerCommandWithStop(index).withTimeout(7.5),
                            new RotateShooterCommand(rot, 70)
                        ),
                        // new AutoShooterCommand(rot, shooter, index, ShooterPositions.SAFE_ZONE),
                        new CustomShooterCommand(rot, shooter, 70, 49, 0.75, 1.5, 2.5).withTimeout(3)
                        // new AutoCustomAngle(rot, shooter, ShooterPositions.WALL_AREA)
                    ),
                    new SequentialCommandGroup(
                        new FollowPath(drive, "close four ring p1"),
                        new InstantCommand(() -> drive.stopModules())
                        // new InstantCommand(() -> index.setCustomSpeed(0.90))
                    )

                ),

                // new InstantCommand(() -> drive.stopModules()),
                //new WaitCommand(1),
                new ParallelDeadlineGroup (
                    // new WaitCommand(0.05),
                    new IndexWithTime(index, 1.7, 0.9),
                    new WaitCommand(0.1),
                    // new InstantCommand(() -> i.stopFeed()),
                    new CustomShooterCommand(rot, shooter, 70, 49, 0.75, 1.5, 2.5).withTimeout(3)
                ),
                // new IndexWithTime(index, 0.5, 0.9),
                // new WaitCommand(0.1),

                new ParallelDeadlineGroup(
                    //new AutoSetIntakeState(intake, frc.robot.commands.AutoSetIntakeState.State.ON),
                    new SequentialCommandGroup(
                        new FollowPath(drive, "close four ring p2", 0.2),
                        new FollowPath(drive, "close four ring p3", new PIDConstants(0.6), 0.0),
                        new InstantCommand(() -> index.setCustomSpeed(0.90))
                    ),
                    new SequentialCommandGroup(
                        // new CustomShooterCommand(rot, shooter, 70, 49, 0.75, 1.5, 2.5).withTimeout(3),
                        // new ParallelCommandGroup(
                        //     // new InstantCommand(() -> i.stopFeed()),
                        //     new InstantCommand(() -> i.setIntakeSpeed(-0.80)),
                        //     new IndexWithTime(index, 1.0, 0.90),
                        //     new WaitCommand(0.01)
                        // ),
                        new ParallelCommandGroup(
                            new InstantCommand(() -> i.setIntakeSpeed(-0.80)),
                            new SequentialCommandGroup(
                                new WaitCommand(0.15),
                                new ParallelCommandGroup(
                                    // new CustomShooterCommand(rot, shooter, 60, 70),
                                    new RotateShooterCommand(rot, 70),
                                    new BetterIndexerCommandWithStop(index).withTimeout(7)
                                ),
                                new AutoCustomAngle(rot, shooter, ShooterPositions.WALL_AREA)
                                // new RotateShooterCommand(rot, 70)
                            )
                            // new InstantCommand(() -> index.setCustomSpeed(0.90))
                        ),
                        new InstantCommand(() -> i.stopFeed())
                    )

                ),

                new ParallelDeadlineGroup(
                    new IndexWithTime(index, 0.5, 1.0),
                    new AutoCustomAngle(rot, shooter, ShooterPositions.WALL_AREA)
                ),
                //new WaitCommand(1),

                
                new ParallelDeadlineGroup(
                    //new AutoSetIntakeState(intake, frc.robot.commands.AutoSetIntakeState.State.ON),
                    new SequentialCommandGroup(
                        new FollowPath(drive, "close four ring p4", 0.5),
                        new FollowPath(drive, "close four ring p5", new PIDConstants(0.9), 0.0),
                        new InstantCommand(() -> drive.stopModules())
                    ),
                    new SequentialCommandGroup(
                        new WaitCommand(0.5),
                        new ParallelDeadlineGroup(
                            new BetterIndexerCommandWithStop(index).withTimeout(7),
                            new RotateShooterCommand(rot, 70)
                        ),
                        new InstantCommand(() -> i.stopFeed()),
                        new AutoCustomAngle(rot, shooter, ShooterPositions.WALL_AREA)
                    ),
                    new InstantCommand(() -> i.setIntakeSpeed(-0.80))
                ),

                new SequentialCommandGroup(
                    new AutoCustomAngle(rot, shooter, ShooterPositions.WALL_AREA),
                    new IndexWithTime(index, 0.5)
                ),

                // new ParallelCommandGroup(
                //     //new AutoSetIntakeState(intake, frc.robot.commands.AutoSetIntakeState.State.ON),
                //     new InstantCommand(() -> i.setIntakeSpeed(-0.80)),
                //     new SequentialCommandGroup(
                //         new WaitCommand(0.5),
                //         new ParallelDeadlineGroup(
                //             new BetterIndexerCommandWithStop(index).withTimeout(5),
                //             new RotateShooterCommand(rot, 70)
                //         )
                //     ),
                //     new SequentialCommandGroup(
                //         new FollowPath(drive, "close four ring p4", 0.5),
                //         new FollowPath(drive, "close four ring p5", new PIDConstants(0.6), 0.0),
                //         new InstantCommand(() -> drive.stopModules())
                //     )

                // ),

                // new ParallelCommandGroup(
                //     new SequentialCommandGroup(
                //         new FollowPath(drive, "close four ring p5", new PIDConstants(0.6), 0.0),
                //         new InstantCommand(() -> drive.stopModules())
                //     ),
                //     // new AutoShooterCommand(rot, shooter, index, ShooterPositions.SAFE_ZONE),
                //     // new CustomShooterCommand(rot, shooter, 65, 49.7, 0.75, 1.0, 1.5).withTimeout(2.2),
                //     new AutoCustomAngle(rot, shooter, ShooterPositions.WALL_AREA),
                //     new InstantCommand(() -> i.stopFeed())
                // ),
                // new WaitCommand(0.25),
                // new IndexWithTime(index, 1.1),
                // new WaitCommand(0.5),

                new ParallelCommandGroup(
                    // new InstantCommand(() -> i.stopFeed()),
                    new SequentialCommandGroup(
                        new WaitCommand(0.6),
                        new CustomShooterCommand(rot, shooter, 40, 70, 0.75, 3.0, 2.5, false).withTimeout(2)

                    ),
                    new SequentialCommandGroup(
                        new IntakeCommand(i, positions.UPPER, 0),
                        new WaitCommand(1.2),
                        new IntakeCommand(i, positions.LOWER, -0.80),
                        new BetterIndexerCommandWithStop(index)
                        
                    ),
                    new SequentialCommandGroup(
                        new FollowPath(drive, "close four ring p6", new PIDConstants(1.2), 0.1),
                        new InstantCommand(() -> drive.stopModules())
                    )
                ),

                new WaitCommand(2),

                new SequentialCommandGroup(
                    new InstantCommand(
                        () -> {
                            rot.stop();
                            // shooter.stop();
                            index.stop();
                            intake.stopFeed();
                            drive.coast();
                        }))

            )
        );


    }

    

}
