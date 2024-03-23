package frc.robot.commands.drivebase;

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
import frc.robot.commands.IndexWithTime;
import frc.robot.commands.RotateShooterCommand;
import frc.robot.commands.ShooterPositions;
import frc.robot.subsystems.IndexerController;
import frc.robot.subsystems.IntakeController;
import frc.robot.subsystems.ShooterController;
import frc.robot.subsystems.ShooterRotationController;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class Four extends SequentialCommandGroup {
    private SwerveDrivetrain drive;
    private IntakeController intake;
    private IndexerController indexer;
    private ShooterController shooter;
    private ShooterRotationController rotShoot;
    public Four(SwerveDrivetrain drive, IntakeController i, IndexerController index, ShooterController shooter, ShooterRotationController rot) {
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
                        new AutoCustomAngle(rot, shooter, ShooterPositions.WALL_AREA),
                        new SequentialCommandGroup(
                            new WaitCommand(1),
                            new IndexWithTime(index, 1.0),
                            new WaitCommand(0.5),
                            new DisableIndexCommand(index)
                        ),
                        new InstantCommand(() -> SmartDashboard.putString("auto location", "first parallel"))
                    ),
                    //new AutoShooterCommand(rot, shooter, index, ShooterPositions.SAFE_ZONE),

                    new InstantCommand(() -> SmartDashboard.putString("auto location", "start path"))

                ),

                new ParallelCommandGroup(
                    //new AutoSetIntakeState(intake, frc.robot.commands.AutoSetIntakeState.State.ON),
                    new AutoLowerIntake(intake),
                    new InstantCommand(() -> i.setIntakeSpeed(-0.80)),
                    new SequentialCommandGroup(
                        new WaitCommand(0.5),
                        new ParallelDeadlineGroup(
                            new BetterIndexerCommandWithStop(index).withTimeout(5),
                            new RotateShooterCommand(rot, 70)
                        ),
                        // new AutoShooterCommand(rot, shooter, index, ShooterPositions.SAFE_ZONE),
                        new CustomShooterCommand(rot, shooter, 65, 49)
                    ),
                    new SequentialCommandGroup(
                        new FollowPath(drive, "four ring p1"),
                        new InstantCommand(() -> drive.stopModules())
                    )

                ),

                // new InstantCommand(() -> drive.stopModules()),
                //new WaitCommand(1),
                new ParallelCommandGroup(
                    new InstantCommand(() -> i.stopFeed()),
                    new IndexWithTime(index, 1)
                ),

                new ParallelCommandGroup(
                    //new AutoSetIntakeState(intake, frc.robot.commands.AutoSetIntakeState.State.ON),
                    new InstantCommand(() -> i.setIntakeSpeed(-0.80)),
                    new SequentialCommandGroup(
                        new WaitCommand(0.5),
                        new ParallelDeadlineGroup(
                            new BetterIndexerCommandWithStop(index).withTimeout(3),
                            new RotateShooterCommand(rot, 70)
                        )
                    ),
                    new SequentialCommandGroup(
                        new FollowPath(drive, "four ring p2"),
                        new InstantCommand(() -> drive.stopModules())
                    )

                ),

                new ParallelCommandGroup(
                    new SequentialCommandGroup(
                        new FollowPath(drive, "four ring p3"),
                        new InstantCommand(() -> drive.stopModules())
                    ),
                    // new AutoShooterCommand(rot, shooter, index, ShooterPositions.SAFE_ZONE),
                    new CustomShooterCommand(rot, shooter, 65, 49.3, 0.75, 1.0, 1.5).withTimeout(2.2),
                    new InstantCommand(() -> i.stopFeed())
                ),
                //new WaitCommand(1),
                new IndexWithTime(index, 1),

                new ParallelCommandGroup(
                    //new AutoSetIntakeState(intake, frc.robot.commands.AutoSetIntakeState.State.ON),
                    new InstantCommand(() -> i.setIntakeSpeed(-0.80)),
                    new SequentialCommandGroup(
                        new WaitCommand(0.5),
                        new ParallelDeadlineGroup(
                            new BetterIndexerCommandWithStop(index).withTimeout(3),
                            new RotateShooterCommand(rot, 70)
                        )
                    ),
                    new SequentialCommandGroup(
                        new FollowPath(drive, "four ring p4", 0.5),
                        new InstantCommand(() -> drive.stopModules())
                    )

                ),

                new ParallelCommandGroup(
                    new SequentialCommandGroup(
                        new FollowPath(drive, "four ring p5"),
                        new InstantCommand(() -> drive.stopModules())
                    ),
                    // new AutoShooterCommand(rot, shooter, index, ShooterPositions.SAFE_ZONE),
                    new CustomShooterCommand(rot, shooter, 65, 49.7, 0.75, 1.0, 1.5).withTimeout(2.2),
                    new InstantCommand(() -> i.stopFeed())
                ),
                new WaitCommand(0.25),
                new IndexWithTime(index, 2),
                new WaitCommand(3),

                new DisableSubsystems(rot, shooter, index, i)

            )
        );


    }

    

}
