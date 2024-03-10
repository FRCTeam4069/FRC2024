package frc.robot.commands.drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoCustomAngle;
import frc.robot.commands.DisableSubsystems;
import frc.robot.commands.ShooterPositions;
import frc.robot.commands.drivebase.BadPIDCommand;
import frc.robot.commands.drivebase.FollowPath;
import frc.robot.commands.drivebase.Rotate;
import frc.robot.commands.drivebase.ShootFirstRing;
import frc.robot.subsystems.IndexerController;
import frc.robot.subsystems.IntakeController;
import frc.robot.subsystems.ShooterController;
import frc.robot.subsystems.ShooterRotationController;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class TwoNoteNewBlue extends SequentialCommandGroup {
    private SwerveDrivetrain drive;
    private IntakeController intake;
    private IndexerController indexer;
    private ShooterController shooter;
    private ShooterRotationController rotShoot;
    public TwoNoteNewBlue(SwerveDrivetrain drive, IntakeController i, IndexerController index, ShooterController shooter, ShooterRotationController rot) {
        this.drive = drive;
        intake = i;
        indexer = index;
        this.shooter = shooter;
        this.rotShoot = rot;
        addRequirements(drive, intake, shooter, rot, index);

        //drive.setPose(new Pose2d(1.3, 5.55, Rotation2d.fromDegrees(180.0)));

        //PathPlannerPath.fromPathFile("one").getGoalEndState()

        addCommands(
            new SequentialCommandGroup(
                //new InstantCommand(() -> drive.setPose(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(-61.5)))),
                new InstantCommand(() -> drive.setPose(new Pose2d(0.7, 4.45, Rotation2d.fromDegrees(60.0)))),
                //new ShootFirstRing(drive, i, index, shooter, rot),

                //new InstantCommand(() -> drive.setPose(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)))),
                //new InstantCommand(() -> drive.setPose(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(-61.5)))),
                
                new BadPIDCommand(drive, new Pose2d(2.4, 4.45, Rotation2d.fromDegrees(25.0))),

                new WaitCommand(2.5),

                new DisableSubsystems(rot, shooter, index, i)

            )
        );


    }

    

}
