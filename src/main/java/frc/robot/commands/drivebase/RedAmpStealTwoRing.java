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

public class RedAmpStealTwoRing extends SequentialCommandGroup {
    private SwerveDrivetrain drive;
    private IntakeController intake;
    private IndexerController indexer;
    private ShooterController shooter;
    private ShooterRotationController rotShoot;
    public RedAmpStealTwoRing(SwerveDrivetrain drive, IntakeController i, IndexerController index, ShooterController shooter, ShooterRotationController rot) {
        this.drive = drive;
        intake = i;
        indexer = index;
        this.shooter = shooter;
        this.rotShoot = rot;
        addRequirements(drive, intake, shooter, rot, index);

        addCommands(
            new SequentialCommandGroup(
                new InstantCommand(() -> drive.setPose(new Pose2d(1.46, 3.77, Rotation2d.fromDegrees(0.0)))),
                new InstantCommand(() -> drive.setPose(new Pose2d(1.46, 3.77, Rotation2d.fromDegrees(0.0)))),

                // new ParallelCommandGroup(
                //     new SequentialCommandGroup(
                //         new FollowPath(drive, "blue amp steal p1", new PIDConstants(0.65), 0.0)
                //         // new InstantCommand(() -> drive.stopModules())
                //     ),
                //     new ParallelCommandGroup(
                //         // new AutoCustomAngle(rot, shooter, ShooterPositions.WALL_AREA),
                //         new CustomShooterCommand(rot, shooter, 20, 70, 0.75, 3.0, 5, true).withTimeout(2),
                //         new SequentialCommandGroup(
                //             new WaitCommand(0.9),
                //             new IndexWithTime(index, 1.0),
                //             new WaitCommand(0.3)
                //         ),
                //         new InstantCommand(() -> SmartDashboard.putString("auto location", "first parallel"))
                //     )
                // ),

                new SequentialCommandGroup(
                    new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                            new FollowPath(drive, "red amp steal p1 and 2", new PIDConstants(4.5), new PIDConstants(1.0), 0.0),
                            // new FollowPath(drive, "blue amp steal p2", new PIDConstants(1.2), 0.5),
                            new FollowPath(drive, "red amp steal p3", new PIDConstants(1.8), 0.1),
                            new InstantCommand(() -> index.setCustomSpeed(0.90))
                        ),
                        new SequentialCommandGroup(
                            new CustomShooterCommand(rot, shooter, 20, 70, 0.75, 3.0, 5, true).withTimeout(2),
                            new SequentialCommandGroup(
                                new WaitCommand(0.2),
                                new IndexWithTime(index, 1.0),
                                new WaitCommand(0.5)
                            ),
                            new IntakeCommand(i, positions.LOWER, -0.80),
                            new ParallelDeadlineGroup(
                                new BetterIndexerCommandWithStop(index).withTimeout(7)
                            )
                        ),
                        new CustomShooterCommand(rot, shooter, 80, 70, 0.75, 3.0, 2.5, false).withTimeout(2)
                    )
                ),
                new ParallelCommandGroup(
                    new SequentialCommandGroup(
                        new FollowPath(drive, "red amp steal further p4", new PIDConstants(2.9), 0.1),
                        new FollowPath(drive, "red amp steal further p5", new PIDConstants(2.9), new PIDConstants(3.0), 0.1),
                        new InstantCommand(() -> drive.stopModules())
                        // new InstantCommand(() -> index.setCustomSpeed(0.90))

                    ),
                    new ParallelCommandGroup(
                        new SequentialCommandGroup(
                            new IndexWithTime(index, 1.0, 0.90),
                            new WaitCommand(0.2),
                            new BetterIndexerCommandWithStop(index).withTimeout(5)
                        ),
                        new CustomShooterCommand(rot, shooter, 80, 70, 0.75, 3.0, 2.5, false).withTimeout(2),
                        new IntakeCommand(i, positions.LOWER, -0.80)
                    )

                ),

                new SequentialCommandGroup(
                    new IntakeCommand(i, positions.UPPER, 0.0),
                    new CustomShooterCommand(rot, shooter, 80, 70, 0.75, 3.0, 2.5, false).withTimeout(2),
                    new IndexWithTime(index, 1.0, 0.80),
                    new WaitCommand(0.3)
                ),

                new WaitCommand(1),

                new DisableSubsystems(rot, shooter, index, i)

            )
        );


    }

    

}
