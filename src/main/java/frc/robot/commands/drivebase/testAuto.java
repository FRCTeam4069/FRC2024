package frc.robot.commands.drivebase;

import com.pathplanner.lib.path.PathPlannerPath;

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

public class testAuto extends SequentialCommandGroup {
    private SwerveDrivetrain drive;
    private IntakeController intake;
    private IndexerController indexer;
    private ShooterController shooter;
    private ShooterRotationController rotShoot;
    public testAuto(SwerveDrivetrain drive, IntakeController i, IndexerController index, ShooterController shooter, ShooterRotationController rot) {
        this.drive = drive;
        intake = i;
        indexer = index;
        this.shooter = shooter;
        this.rotShoot = rot;
        addRequirements(drive, intake, shooter, rot, index);

        drive.setPose(new Pose2d(1.3, 5.55, Rotation2d.fromDegrees(180.0)));

        //PathPlannerPath.fromPathFile("one").getGoalEndState()

        addCommands(
            new SequentialCommandGroup(
                new InstantCommand(() -> drive.setPose(new Pose2d(1.3, 5.55, Rotation2d.fromDegrees(180.0)))),
                new ShootFirstRing(drive, i, index, shooter, rot),
                
                new FollowPath(drive, "one"),

                new InstantCommand(() -> SmartDashboard.putString("auto location", "end path")),
                new WaitCommand(2.5),

                // new InstantCommand(() -> drive.setPose(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(155.0)))),
                //new InstantCommand(() -> drive.setPose(new Pose2d(2.388, 4.392, Rotation2d.fromRadians(-drive.getRadians())))),

                new Rotate(drive, 0),
                new InstantCommand(() -> drive.setPose(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(drive.getDegrees())))),
                new BadPIDCommand(drive, new Pose2d(-0.5, 0.0, Rotation2d.fromDegrees(drive.getDegrees()))),
                new BadPIDCommand(drive, new Pose2d(-0.5, 1.45, Rotation2d.fromDegrees(drive.getDegrees()))),
                new BadPIDCommand(drive, new Pose2d(0.0, 1.45, Rotation2d.fromDegrees(drive.getDegrees()))),

                //new FollowPath(drive, "bad two"),
                new WaitCommand(4),

                new DisableSubsystems(rot, shooter, index, i)

            )
        );


    }

}
