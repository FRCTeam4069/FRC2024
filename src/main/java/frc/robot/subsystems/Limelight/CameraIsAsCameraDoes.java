package frc.robot.subsystems.Limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraIsAsCameraDoes extends SubsystemBase{
    
String cameraName;
LimelightHelpers limeLight = new LimelightHelpers();

    public CameraIsAsCameraDoes(String camName) {
        cameraName = camName;

    }

    public double getXDistanceToApriltag(int tagNumber) {
        if (limeLight.getFiducialID(cameraName) == tagNumber) {
            return limeLight.getTargetPose3d_RobotSpace(cameraName).getZ();
        }
        else return 0;
    }
    
    public double getYDistanceToApriltag(int tagNumber) {
        if (limeLight.getFiducialID(cameraName) == tagNumber) {
            return limeLight.getTargetPose3d_RobotSpace(cameraName).getX();
        }
        else return 0;
    }

    public Pose2d getRobotPose2d() {
        return limeLight.getLatestResults(cameraName).targetingResults.getBotPose2d();
    }

    public void printNumbers() {
        SmartDashboard.putNumber(cameraName + " fiducial ID: ", limeLight.getFiducialID(cameraName));

        SmartDashboard.putNumber(cameraName + " X from center of field(meter): ", limeLight.getLatestResults(cameraName).targetingResults.getBotPose2d().getX());
        SmartDashboard.putNumber(cameraName + " Y from center of field(meter): ", limeLight.getLatestResults(cameraName).targetingResults.getBotPose2d().getY());
        SmartDashboard.putNumber(cameraName + " Rotation from tag(degrees): ", limeLight.getLatestResults(cameraName).targetingResults.getBotPose2d().getRotation().getDegrees());

        SmartDashboard.putNumber(cameraName + " Z from april tag(meter)", limeLight.getTargetPose3d_RobotSpace(cameraName).getZ());
        SmartDashboard.putNumber(cameraName + " X from april tag(meters)", limeLight.getTargetPose3d_RobotSpace(cameraName).getZ());
        
        SmartDashboard.putNumber(cameraName + " X from center of blue alliance side(meter): ", limeLight.getLatestResults(cameraName).targetingResults.getBotPose3d_wpiBlue().getX());
        SmartDashboard.putNumber(cameraName + " Y from center of blue alliance side(meter): ", limeLight.getLatestResults(cameraName).targetingResults.getBotPose3d_wpiBlue().getY());
    }
}
