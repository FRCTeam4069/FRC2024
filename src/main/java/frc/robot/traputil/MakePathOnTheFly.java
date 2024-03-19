package frc.robot.traputil;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.Limelight.PoseEstimatorSubsystem;

public class MakePathOnTheFly extends Command {

    Pose2d targetPose;
    double targetEndVelocity;
    boolean rotateFast;
    PathConstraints constraints;
    PoseEstimatorSubsystem poseEstimator;
    Alliance alliance;

    public MakePathOnTheFly(double targetEndVelocity, boolean rotateFast, PathConstraints constraints, PoseEstimatorSubsystem poseEstimator, Alliance alliance) {
        this.targetEndVelocity = targetEndVelocity;
        this.rotateFast = rotateFast;
        this.constraints = constraints;
        this.poseEstimator = poseEstimator;
        this.alliance = alliance;
    }

    public void initialize() {
        if (alliance == Alliance.Blue) {
            targetPose = FieldConstants.blueTrap;
        }
        else if (alliance == Alliance.Red) {
            targetPose = FieldConstants.redTrap;
        }

        final List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                poseEstimator.getCurrentPose(),
                targetPose
        );

        final PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                new PathConstraints(5.0, 1.0, 1.0, 0.5),
                new GoalEndState(targetEndVelocity, targetPose.getRotation(), rotateFast)
        );

        path.preventFlipping = true;
        
        AutoBuilder.followPath(path);
    }

    public void execute() {

    }

    public void end(boolean interrupted) {
        
    }

    public boolean isFinished() {
        return false;
    }
}