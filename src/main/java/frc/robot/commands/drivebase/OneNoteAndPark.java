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

public class OneNoteAndPark extends SequentialCommandGroup {
    private SwerveDrivetrain drive;
    private IntakeController intake;
    private IndexerController indexer;
    private ShooterController shooter;
    private ShooterRotationController rotShoot;
    public OneNoteAndPark(SwerveDrivetrain drive, IntakeController i, IndexerController index, ShooterController shooter, ShooterRotationController rot) {
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
                        new Rotate(drive, Units.degreesToRadians(-34), Units.degreesToRadians(1)).withTimeout(6),
                        new CustomShooterCommand(rot, shooter, 58, 40)
                        //new AutoCustomAngle(rot, shooter, ShooterPositions.WALL_AREA)
                    ),
                    new IndexWithTime(index, 1.2),
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
                        new Rotate(drive, 0).withTimeout(5),
                        new InstantCommand(() -> drive.stopModules())
                    )
                ),

                new WaitCommand(3),
                
                new BadPIDCommand(drive, new Pose2d(2.0, 6.50, new Rotation2d())),


                new DisableSubsystems(rot, shooter, index, i)

            )
        );


    }

    

}
