package frc.robot.commands.drivebase.test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoCustomAngle;
import frc.robot.commands.AutoLowerIntake;
import frc.robot.commands.AutoSetIntakeState;
import frc.robot.commands.DisableSubsystems;
import frc.robot.commands.ShooterPositions;
import frc.robot.commands.drivebase.BadPIDCommand;
import frc.robot.commands.drivebase.FollowPath;
import frc.robot.commands.drivebase.Rotate;
import frc.robot.commands.drivebase.ShootFirstRing;
import frc.robot.constants.DrivebaseConstants;
import frc.robot.subsystems.IndexerController;
import frc.robot.subsystems.IntakeController;
import frc.robot.subsystems.ShooterController;
import frc.robot.subsystems.ShooterRotationController;
import frc.robot.subsystems.swerve.SwerveDrivetrain;

public class DriveTest extends SequentialCommandGroup {
    public DriveTest(SwerveDrivetrain drive) {
        addRequirements(drive);

        drive.setPose(new Pose2d(1.3, 5.55, Rotation2d.fromDegrees(0.0)));

        addCommands(
            new SequentialCommandGroup(
                new ParallelRaceGroup(
                    new RunCommand(() -> drive.drive(new ChassisSpeeds(0, DrivebaseConstants.maxVelocity/2, 0))),
                    new WaitCommand(5)
                )

            )
        );


    }

    

}
