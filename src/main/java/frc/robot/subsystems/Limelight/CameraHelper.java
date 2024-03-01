package frc.robot.subsystems.Limelight;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraHelper extends SubsystemBase {

    private final PhotonCamera cam;
    private static PhotonPoseEstimator photonPoseEstimator;
    private PhotonPipelineResult result;
    private PhotonTrackedTarget target;

    private Transform3d centerToCamera;

    public CameraHelper(final String camName, final AprilTagFieldLayout layout, final Transform3d centerToCamera) {
        this.cam = new PhotonCamera(camName);
        this.result = new PhotonPipelineResult();
        // this.target = new PhotonTrackedTarget();

        photonPoseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, centerToCamera);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public final void update() {
        result = cam.getLatestResult();
        if (result.hasTargets()) target = result.getBestTarget();
    }
    
    public final PhotonPipelineResult getResult() {
        return result;
    }

    public final PhotonTrackedTarget getTarget() {
        return target;
    }

    public final boolean hasTargets() {
        return result.hasTargets();
    }

    public final int getNumberOfTargets() {
        return result.getTargets().size();
    }

    public final Optional<Transform3d> getTransformToAprilTag3d(final double poseAmbiguity) {
        if (target == null) return Optional.empty();
        final Transform3d transform = target.getBestCameraToTarget().inverse();
        final Transform3d adjusted = transform.plus(centerToCamera);
        if (!(target.getPoseAmbiguity() <= poseAmbiguity && target.getPoseAmbiguity() != -1 &&
              target.getFiducialId() >= 0))
            return Optional.empty();
        return Optional.of(adjusted);
    }

    

}
