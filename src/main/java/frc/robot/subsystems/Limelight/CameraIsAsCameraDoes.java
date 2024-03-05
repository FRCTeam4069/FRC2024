package frc.robot.subsystems.Limelight;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.CameraConstants;

public class CameraIsAsCameraDoes extends SubsystemBase {

    String cameraName;
    LimelightHelpers limeLight = new LimelightHelpers();

    LinearFilter filter = LinearFilter.movingAverage(10);
    double lastgoodX = 0; // initial
    double lastgoodY = 0; // initial
    Rotation3d lastgoodHeading = new Rotation3d(); // initial

    public CameraIsAsCameraDoes(String camName) {
        cameraName = camName;

    }

    public double getXDistanceToApriltag(int a, int b) {
        if (a == limeLight.getFiducialID(cameraName) || b == limeLight.getFiducialID(cameraName)) {
                lastgoodX = filter.calculate(limeLight.getTargetPose3d_RobotSpace(cameraName).getZ());
        }
        return lastgoodX;
    }

    public double getXDistanceToApriltag(int tagNumber) {
        if (LimelightHelpers.getFiducialID(cameraName) == tagNumber) {
            lastgoodX = LimelightHelpers.getTargetPose3d_RobotSpace(cameraName).getZ();
        }
        return lastgoodX;

    }

    public double getYDistanceToApriltag(int tagNumber) {
        if (LimelightHelpers.getFiducialID(cameraName) == tagNumber) {
            lastgoodY = LimelightHelpers.getTargetPose3d_RobotSpace(cameraName).getX();
        } 
        return lastgoodY;
    }

    public Translation2d getTargetTranslation(int tagNumber) {
        var y = getYDistanceToApriltag(tagNumber);
        var x = getXDistanceToApriltag(tagNumber);
        return new Translation2d(x, y);

    }

    public Rotation3d getTargetRotation() {
        if (LimelightHelpers.getFiducialID(cameraName) == 7) {
            lastgoodHeading = LimelightHelpers.getTargetPose3d_RobotSpace(cameraName).getRotation();
        } else if (LimelightHelpers.getFiducialID(cameraName) == 8) {
            var translation = LimelightHelpers.getTargetPose3d_RobotSpace(cameraName).getTranslation().minus(new Translation3d(0, 0, 0.5));
            lastgoodHeading = new Rotation3d(0.0, Math.atan2(translation.getX(), translation.getZ()),  0.0);

        } else {

        }
        return lastgoodHeading;
        //return LimelightHelpers.getTargetPose3d_RobotSpace(cameraName).getRotation();
        //return LimelightHelpers.getLatestResults(cameraName).targetingResults.getBotPose2d().getRotation();
    }

    public Pose2d getRobotPose2d() {
        return limeLight.getLatestResults(cameraName).targetingResults.getBotPose2d();
    }

    public void printNumbers() {
        SmartDashboard.putNumber(cameraName + " fiducial ID: ", limeLight.getFiducialID(cameraName));

        SmartDashboard.putNumber(cameraName + " X from center of field(meter): ",
                limeLight.getLatestResults(cameraName).targetingResults.getBotPose2d().getX());
        SmartDashboard.putNumber(cameraName + " Y from center of field(meter): ",
                limeLight.getLatestResults(cameraName).targetingResults.getBotPose2d().getY());
        SmartDashboard.putNumber(cameraName + " Rotation from tag(degrees): ",
                limeLight.getLatestResults(cameraName).targetingResults.getBotPose2d().getRotation().getDegrees());

        SmartDashboard.putNumber(cameraName + " Z from april tag(meter)",
                limeLight.getTargetPose3d_RobotSpace(cameraName).getZ());
        SmartDashboard.putNumber(cameraName + " X from april tag(meters)",
                limeLight.getTargetPose3d_RobotSpace(cameraName).getX());

        SmartDashboard.putNumber(cameraName + " X from center of blue alliance side(meter): ",
                limeLight.getLatestResults(cameraName).targetingResults.getBotPose3d_wpiBlue().getX());
        SmartDashboard.putNumber(cameraName + " Y from center of blue alliance side(meter): ",
                limeLight.getLatestResults(cameraName).targetingResults.getBotPose3d_wpiBlue().getY());
    }
}
