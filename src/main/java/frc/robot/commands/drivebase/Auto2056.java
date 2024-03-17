package frc.robot.commands.drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoCustomAngle;
import frc.robot.commands.DisableSubsystems;
import frc.robot.commands.ShooterPositions;
import frc.robot.subsystems.IndexerController;
import frc.robot.subsystems.IntakeController;
import frc.robot.subsystems.ShooterController;
import frc.robot.subsystems.ShooterRotationController;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class Auto2056 extends SequentialCommandGroup {
    private SwerveDrivetrain drive;
    private IntakeController intake;
    private IndexerController indexer;
    private ShooterController shooter;
    private ShooterRotationController rotShoot;
    public Auto2056(SwerveDrivetrain drive, IntakeController i, IndexerController index, ShooterController shooter, ShooterRotationController rot) {
        this.drive = drive;
        intake = i;
        indexer = index;
        this.shooter = shooter;
        this.rotShoot = rot;
        addRequirements(drive, intake, shooter, rot, index);

        addCommands(
            new SequentialCommandGroup(
                new InstantCommand(() -> drive.setPose(new Pose2d(0.65, 6.64, Rotation2d.fromDegrees(60.0)))),
                //new ShootFirstRing(drive, i, index, shooter, rot),
                // new FollowPath(drive, "start left align"),
                // new WaitCommand(3),
                // new FollowPath(drive, "start left close ring"),
                // new WaitCommand(3),

                //new FollowPath(drive, "start very left to ring"),
                new FollowPath(drive, "New New Path"),

                //new WaitCommand(3),

                //new FollowPath(drive, "close to left"),

                // new InstantCommand(() -> SmartDashboard.putString("auto location", "end path")),
                // new WaitCommand(1.5),

                // // new InstantCommand(() -> drive.setPose(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(155.0)))),
                // //new InstantCommand(() -> drive.setPose(new Pose2d(2.388, 4.392, Rotation2d.fromRadians(-drive.getRadians())))),

                // new Rotate(drive, 0).withTimeout(1),
                // new InstantCommand(() -> drive.setPose(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(drive.getDegrees())))),

                // new BadPIDCommand(drive, new Pose2d(-0.5, 0.0, Rotation2d.fromDegrees(drive.getDegrees()))),
                

                // new ParallelCommandGroup(

                //     new AutoCustomAngle(rot, shooter, ShooterPositions.CLIMB),
                //     new SequentialCommandGroup(
                //         new BadPIDCommand(drive, new Pose2d(-0.5, 1.54, Rotation2d.fromDegrees(drive.getDegrees()))),
                //         new BadPIDCommand(drive, new Pose2d(0.3, 1.54, Rotation2d.fromDegrees(drive.getDegrees())))

                //     )
                // ),

                // new WaitCommand(3.5),

                // // new BadPIDCommand(drive, new Pose2d(-0.5, 1.12, Rotation2d.fromDegrees(drive.getDegrees()))),
                // // new BadPIDCommand(drive, new Pose2d(-0.5, 2.50, Rotation2d.fromDegrees(drive.getDegrees()))),
                // // new Rotate(drive, Units.degreesToRadians(20)).withTimeout(1.5),

                // // new BadPIDCommand(drive, new Pose2d(-0.5, 2.50, Rotation2d.fromDegrees(200.0))),
                // // new BadPIDCommand(drive, new Pose2d(0.3, 2.50, Rotation2d.fromDegrees(200.0))),
                // // //new FollowPath(drive, "bad two"),
                // // new WaitCommand(3),

                new DisableSubsystems(rot, shooter, index, i)

            )
        );


    }

    

}
