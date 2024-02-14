// uncomment when used 

package frc.robot.subsystems.Limelight;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.CameraConstants;

public class CameraController extends SubsystemBase {
    private PhotonCamera cam;
    private static PhotonPoseEstimator photonPoseEstimator;
    private RobotContainer m_RobotContainer;
    private String tableName;
    
    public CameraController(String camName) {
        this.cam = new PhotonCamera(camName);
        photonPoseEstimator = new PhotonPoseEstimator(m_RobotContainer.aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, cam, CameraConstants.robotToFrontCam);
    }

    public PhotonPipelineResult getResult() {
         return cam.getLatestResult();
    }

    public boolean hasTarget() {
        return getResult().hasTargets();
    }

    public List<PhotonTrackedTarget> getTarget() {
        if (hasTarget()) {
            return getResult().getTargets();
        }
        else return null;
    }

    public PhotonTrackedTarget getBestTarget() {
        return  ((PhotonPipelineResult) getTarget()).getBestTarget();
    }

    public double getYaw() {
        return getBestTarget().getYaw();
    }

    public double getPitch() {
        return getBestTarget().getPitch();
    }

    public double getArea() {
        return getBestTarget().getArea();
    }

    public int getTargetID() {
        return getBestTarget().getFiducialId();
    }

    public double getPoseAmbiguity() {
        return getBestTarget().getPoseAmbiguity();
    }

    public Transform3d getbestCamToTarget() {
        return getBestTarget().getBestCameraToTarget();
    }

    public Transform3d getAltCamToTarget() {
        return getBestTarget().getAlternateCameraToTarget();
    }

    public double getLatency() {
        return getResult().getLatencyMillis();
    }

    public static Optional<EstimatedRobotPose> getEstimatedPose() {
        return photonPoseEstimator.update();
    }

    public static double getDistanceToTarget() {
        return PhotonUtils.calculateDistanceToTargetMeters(0, 0, 0, 0);
    }

    public void setLedMode(VisionLEDMode ledMode) {
        this.cam.setLED(ledMode);
    }

    public void setPipeline(int index) {
        this.cam.setPipelineIndex(index);
    }

    public void setDriver(boolean driverMode) {
        this.cam.setDriverMode(driverMode);
    }

    public void printNumbers() {
        SmartDashboard.putNumber(tableName + " targetArea:", getArea());
        SmartDashboard.putNumber(tableName + " targetPitch:", getPitch());
        SmartDashboard.putNumber(tableName + " targetYaw:", getYaw());
        SmartDashboard.putNumber(tableName + " targetDistance:", getDistanceToTarget());
    }

}
             