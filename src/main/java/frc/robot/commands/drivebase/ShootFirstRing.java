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
import frc.robot.commands.DisableIndexCommand;
import frc.robot.commands.DisableSubsystems;
import frc.robot.commands.IndexWithTime;
import frc.robot.commands.ShooterPositions;
import frc.robot.subsystems.IndexerController;
import frc.robot.subsystems.IntakeController;
import frc.robot.subsystems.ShooterController;
import frc.robot.subsystems.ShooterRotationController;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class ShootFirstRing extends SequentialCommandGroup {
    private IntakeController intake;
    private IndexerController indexer;
    private ShooterController shooter;
    private ShooterRotationController rotShoot;
    public ShootFirstRing(IntakeController i, IndexerController index, ShooterController shooter, ShooterRotationController rot) {
        intake = i;
        indexer = index;
        this.shooter = shooter;
        this.rotShoot = rot;
        addRequirements(intake, shooter, rot, index);


        addCommands(
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
                new AutoShooterCommand(rot, shooter, index, ShooterPositions.SAFE_ZONE),

                new ParallelCommandGroup(
                    new AutoLowerIntake(intake),
                    new InstantCommand(() -> SmartDashboard.putString("auto location", "second parallel"))
                ),

                new InstantCommand(() -> SmartDashboard.putString("auto location", "start path")),

                new ParallelCommandGroup(
                    new AutoSetIntakeState(intake, frc.robot.commands.AutoSetIntakeState.State.ON),
                    new IndexWithTime(index, 1.4)
                )

            )
        );


    }

}
