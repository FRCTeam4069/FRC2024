package frc.robot.subsystems.Limelight;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.CameraConstants;
import frc.robot.constants.FieldConstants;

public class CameraIsAsCameraDoes extends SubsystemBase {

    String cameraName;
    LimelightHelpers limeLight = new LimelightHelpers();

    LinearFilter filter = LinearFilter.movingAverage(15);
    double lastgoodX = 0; // initial
    double lastgoodY = 0; // initial
    Rotation3d lastgoodHeading = new Rotation3d(); // initial
    double tx = 0;
    LinearFilter txFilter = LinearFilter.movingAverage(10);

    public CameraIsAsCameraDoes(String camName) {
        cameraName = camName;

    }

    public double getXDistanceToApriltag(int a, int b) {
        if (a == limeLight.getFiducialID(cameraName) || b == limeLight.getFiducialID(cameraName)) {
                lastgoodX = filter.calculate(limeLight.getCameraPose3d_TargetSpace(cameraName).getZ());
        }
        return lastgoodX;
    }

    public double getXDistanceToApriltag(int tagNumber) {
        if (LimelightHelpers.getFiducialID(cameraName) == tagNumber) {
            lastgoodX = LimelightHelpers.getCameraPose3d_TargetSpace(cameraName).getZ();
        }
        return lastgoodX;

    }

    public double getYDistanceToApriltag(int a, int b) {
       if (a == limeLight.getFiducialID(cameraName) || b == limeLight.getFiducialID(cameraName)) {
                lastgoodY = filter.calculate(limeLight.getCameraPose3d_TargetSpace(cameraName).getX());
        }

        return lastgoodY;
    }

    /**
     * 
     * @param a
     * @param b
     * @return radians
     */
    public double getTX(int a, int b) {
        var id = LimelightHelpers.getFiducialID(cameraName);
        var newTX = LimelightHelpers.getTX(cameraName);

        if (id == a || id == b) {
            if (!(newTX == 0.0 && Math.abs(tx) > 10)) {
                tx = txFilter.calculate(LimelightHelpers.getTX(cameraName));
            }
        }
        return Units.degreesToRadians(tx);
    }

    public boolean hasTarget(int a, int b) {
        var id = LimelightHelpers.getFiducialID(cameraName);
        return (id == a || id == b);

    }

    /**
     * @return radians
     */
    public double getAngleDifferently() {
        var pose = LimelightHelpers.getBotPose2d_wpiBlue(cameraName);
        var diff = pose.minus(FieldConstants.poseBlueSpeaker);
        var angle = Math.atan2(diff.getY(), diff.getX());
        SmartDashboard.putNumber("angle from speaker", angle);
        SmartDashboard.putNumber("angle from speaker(deg)", Units.radiansToDegrees(angle));
        SmartDashboard.putNumber("field pose x", pose.getX());
        SmartDashboard.putNumber("field pose y", pose.getY());
        SmartDashboard.putNumber("robotospeaker pose x", diff.getX());
        SmartDashboard.putNumber("robotospeaker pose y", diff.getY());

        return angle;

    }

    // public Translation2d getTargetTranslation(int tagNumber) {

    //
    // }

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
        SmartDashboard.putNumber("Limelight X", getXDistanceToApriltag(4, 7));
        SmartDashboard.putNumber("Limelight Y", getYDistanceToApriltag(4, 7));
        //SmartDashboard.putNumber(cameraName, lastgoodX)

        // SmartDashboard.putNumber(cameraName + " X from center of field(meter): ",
        //         limeLight.getLatestResults(cameraName).targetingResults.getBotPose2d().getX());
        // SmartDashboard.putNumber(cameraName + " Y from center of field(meter): ",
        //         limeLight.getLatestResults(cameraName).targetingResults.getBotPose2d().getY());
        // SmartDashboard.putNumber(cameraName + " Rotation from tag(degrees): ",
        //         limeLight.getLatestResults(cameraName).targetingResults.getBotPose2d().getRotation().getDegrees());

        // SmartDashboard.putNumber(cameraName + " Z from april tag(meter)",
        //         limeLight.getTargetPose3d_RobotSpace(cameraName).getZ());
        // SmartDashboard.putNumber(cameraName + " X from april tag(meters)",
        //         limeLight.getTargetPose3d_RobotSpace(cameraName).getX());

        // SmartDashboard.putNumber(cameraName + " X from center of blue alliance side(meter): ",
        //         limeLight.getLatestResults(cameraName).targetingResults.getBotPose3d_wpiBlue().getX());
        // SmartDashboard.putNumber(cameraName + " Y from center of blue alliance side(meter): ",
        //         limeLight.getLatestResults(cameraName).targetingResults.getBotPose3d_wpiBlue().getY());
    }
}
