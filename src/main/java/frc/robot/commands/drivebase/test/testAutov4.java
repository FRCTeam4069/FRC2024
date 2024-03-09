package frc.robot.commands.drivebase.test;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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
import frc.robot.commands.drivebase.BadPIDCommand;
import frc.robot.commands.drivebase.FollowPath;
import frc.robot.commands.drivebase.Rotate;
import frc.robot.commands.drivebase.ShootFirstRing;
import frc.robot.subsystems.IndexerController;
import frc.robot.subsystems.IntakeController;
import frc.robot.subsystems.ShooterController;
import frc.robot.subsystems.ShooterRotationController;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class testAutov4 extends SequentialCommandGroup {
    private SwerveDrivetrain drive;
    private IntakeController intake;
    private IndexerController indexer;
    private ShooterController shooter;
    private ShooterRotationController rotShoot;
    public testAutov4(SwerveDrivetrain drive, IntakeController i, IndexerController index, ShooterController shooter, ShooterRotationController rot) {
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
                
                new FollowPath(drive, "one v4"),

                new InstantCommand(() -> SmartDashboard.putString("auto location", "end path")),
                new WaitCommand(2.5),

                // new InstantCommand(() -> drive.setPose(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(155.0)))),
                //new InstantCommand(() -> drive.setPose(new Pose2d(2.388, 4.392, Rotation2d.fromRadians(-drive.getRadians())))),

                new Rotate(drive, 0).withTimeout(1),
                new InstantCommand(() -> drive.setPose(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(drive.getDegrees())))),
                new BadPIDCommand(drive, new Pose2d(-0.5, 0.0, Rotation2d.fromDegrees(drive.getDegrees()))),
                new BadPIDCommand(drive, new Pose2d(-0.5, 1.24, Rotation2d.fromDegrees(drive.getDegrees()))),
                new BadPIDCommand(drive, new Pose2d(0.3, 1.24, Rotation2d.fromDegrees(drive.getDegrees()))),
                new WaitCommand(3.5),

                // new BadPIDCommand(drive, new Pose2d(-0.5, 1.12, Rotation2d.fromDegrees(drive.getDegrees()))),
                // new BadPIDCommand(drive, new Pose2d(-0.5, 2.50, Rotation2d.fromDegrees(drive.getDegrees()))),
                // new Rotate(drive, Units.degreesToRadians(20)).withTimeout(1.5),

                // new BadPIDCommand(drive, new Pose2d(-0.5, 2.50, Rotation2d.fromDegrees(200.0))),
                // new BadPIDCommand(drive, new Pose2d(0.3, 2.50, Rotation2d.fromDegrees(200.0))),
                // //new FollowPath(drive, "bad two"),
                // new WaitCommand(3),

                new DisableSubsystems(rot, shooter, index, i)

            )
        );


    }

    

}
