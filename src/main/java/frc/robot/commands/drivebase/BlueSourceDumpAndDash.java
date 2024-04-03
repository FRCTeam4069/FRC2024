package frc.robot.commands.drivebase;

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
import frc.robot.subsystems.IndexerController;
import frc.robot.subsystems.IntakeController;
import frc.robot.subsystems.ShooterController;
import frc.robot.subsystems.ShooterRotationController;
import frc.robot.subsystems.IntakeController.positions;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class BlueSourceDumpAndDash extends SequentialCommandGroup {
    private SwerveDrivetrain drive;
    private IntakeController intake;
    private IndexerController indexer;
    private ShooterController shooter;
    private ShooterRotationController rotShoot;
    public BlueSourceDumpAndDash(SwerveDrivetrain drive, IntakeController i, IndexerController index, ShooterController shooter, ShooterRotationController rot) {
        this.drive = drive;
        intake = i;
        indexer = index;
        this.shooter = shooter;
        this.rotShoot = rot;
        addRequirements(drive, intake, shooter, rot, index);

        addCommands(
            new SequentialCommandGroup(
                new InstantCommand(() -> drive.setPose(new Pose2d(0.73, 4.48, Rotation2d.fromDegrees(-60.0)))),
                new InstantCommand(() -> drive.setPose(new Pose2d(0.73, 4.48, Rotation2d.fromDegrees(-60.0)))),

                new SequentialCommandGroup(
                    new CustomShooterCommand(rot, shooter, 60, 35, 0.75, 3.0, 10.0),
                    new SequentialCommandGroup(
                        new WaitCommand(0.1),
                        new IndexWithTime(index, 1.0),
                        new WaitCommand(0.3)
                    )
                ),

                new InstantCommand(() -> {rot.stop(); shooter.stop();}),

                new BadPIDCommand(drive, new Pose2d(1.90, 2.10, Rotation2d.fromDegrees(0.0)), 0.02, 2.0),
                new InstantCommand(() -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.fromDegrees(0.0)))),
                new InstantCommand(() -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.fromDegrees(0.0)))),
                new FollowPath(drive, "blue source dump and dash", new PIDConstants(3.0), 0.0),

                // new Rotate(drive, Units.degreesToRadians(60.0), 0.5),

                new DisableSubsystems(rot, shooter, index, i)

            )
        );


    }

    

}
