package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drivebase.CameraPIDCommand;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.IndexerController;
import frc.robot.subsystems.ShooterController;
import frc.robot.subsystems.ShooterRotationController;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class Trap extends SequentialCommandGroup {
    public Trap(SwerveDrivetrain drive, ShooterRotationController shooterRotation, ShooterController shooter, IndexerController indexer) {
        addRequirements(drive, shooterRotation, shooter, indexer);

        addCommands(
            new SequentialCommandGroup(
                new CameraPIDCommand(drive, FieldConstants.blueTrap),
                new InstantCommand(() -> drive.stopModules()),
                new CustomShooterCommand(shooterRotation, shooter, 30, 40.25, 1.0, 0.5, 0.5),
                new WaitCommand(0.5),
                new IndexWithTime(indexer, 1.0, 0.8),
                new WaitCommand(0.5),
                new InstantCommand(() -> {
                    shooterRotation.stop();
                    shooter.stop();
                    indexer.stop();
                })

            )
        );

    }
    
}
