package frc.robot.subsystems.Limelight;

import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide;
import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide;

import java.util.function.Consumer;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;

import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CameraConstants;
// import frc.robot.constants.DrivetrainConstants;
// import frc.robot.util.FieldConstants;
import frc.robot.constants.DrivebaseConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.util.TimedPose2d;

public class PoseEstimatorSubsystem extends SubsystemBase {

    // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
    // you trust your various sensors. Smaller numbers will cause the filter to
    // "trust" the estimate from that particular component more than the others.
    // This in turn means the particualr component will have a stronger influence
    // on the final pose estimate.

    // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
    // you trust your various sensors. Smaller numbers will cause the filter to
    // "trust" the estimate from that particular component more than the others.
    // This in turn means the particualr component will have a stronger influence
    // on the final pose estimate.

    private final Supplier<Rotation2d> rotationSupplier;
    private final Supplier<SwerveModulePosition[]> modulePositionSupplier;
    private final Consumer<TimedPose2d> poseConsumer;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field2d = new Field2d();
    private final PhotonRunnable frontEstimator = new PhotonRunnable(new PhotonCamera("Right_Front"),
            CameraConstants.robotCenterToFrontCam);
    private final PhotonRunnable rightEstimator = new PhotonRunnable(new PhotonCamera("Right_Side"),
            CameraConstants.robotCenterToRightCam);
    private final PhotonRunnable leftEstimator = new PhotonRunnable(new PhotonCamera("Left_Front"),
            CameraConstants.robotCenterToFrontCam);
    private final PhotonRunnable leftSideEstimator = new PhotonRunnable(new PhotonCamera("Left_Side"),
            CameraConstants.robotCenterToRightCam);

    private final Notifier allNotifier = new Notifier(() -> {
        frontEstimator.run();
        rightEstimator.run();
    });

    private OriginPosition originPosition = kBlueAllianceWallRightSide;

    private final StructPublisher<Pose2d> posePublisher;

    // private final ArrayList<Double> xValues = new ArrayList<Double>();
    // private final ArrayList<Double> yValues = new ArrayList<Double>();

    public PoseEstimatorSubsystem(Supplier<Rotation2d> rotationSupplier,
                                  Supplier<SwerveModulePosition[]> modulePositionSupplier, Consumer<TimedPose2d> poseConsumer) {
        this.rotationSupplier = rotationSupplier;
        this.modulePositionSupplier = modulePositionSupplier;
        this.poseConsumer = poseConsumer;

        poseEstimator = new SwerveDrivePoseEstimator(
                DrivebaseConstants.kinematics,
                rotationSupplier.get(),
                modulePositionSupplier.get(),
                new Pose2d(1.3, 5.55, new Rotation2d()),
                CameraConstants.STATE_STANDARD_DEVIATIONS,
                CameraConstants.VISION_MEASUREMENT_STANDARD_DEVIATIONS);

        allNotifier.setName("runAll");
        allNotifier.startPeriodic(0.02);

        posePublisher = NetworkTableInstance.getDefault()
                .getStructTopic("CameraPose", Pose2d.struct).publish();

    }

    public void addDashboardWidgets(ShuffleboardTab tab) {
        tab.add("Field", field2d).withPosition(0, 0).withSize(6, 4);
        tab.addString("Pose", this::getFomattedPose).withPosition(6, 2).withSize(2, 1);
        tab.addString("Speaker Transform", this::getFormattedTransform);
    }

    /**
     * Sets the alliance. This is used to configure the origin of the AprilTag map
     *
     * @param alliance alliance
     */
    public void setAlliance(Alliance alliance) {
        boolean allianceChanged = false;
        switch (alliance) {
            case Blue:
                allianceChanged = (originPosition == kRedAllianceWallRightSide);
                originPosition = kBlueAllianceWallRightSide;
                break;
            case Red:
                allianceChanged = (originPosition == kBlueAllianceWallRightSide);
                originPosition = kRedAllianceWallRightSide;
                break;
            default:
                // No valid alliance data. Nothing we can do about it
        }

        if (allianceChanged) {
            // The alliance changed, which changes the coordinate system.
            // Since a tag was seen, and the tags are all relative to the coordinate system,
            // the estimated pose
            // needs to be transformed to the new coordinate system.
            var newPose = flipAlliance(getCurrentPose());
            poseEstimator.resetPosition(rotationSupplier.get(), modulePositionSupplier.get(), newPose);
        }
    }

    @Override
    public void periodic() {
        // Update pose estimator with drivetrain sensors
        poseEstimator.update(rotationSupplier.get(), modulePositionSupplier.get());
        if (CameraConstants.USE_VISION) {
            estimatorChecker(frontEstimator);
            estimatorChecker(rightEstimator);
            estimatorChecker(leftEstimator);
            estimatorChecker(leftSideEstimator);
        } else {
            allNotifier.close();
        }

        // estimatorChecker(backEstimator);

        // Set the pose on the dashboard
        var dashboardPose = poseEstimator.getEstimatedPosition();
        // if (originPosition == kRedAllianceWallRightSide) {
        //   // Flip the pose when red, since the dashboard field photo cannot be rotated
        //   dashboardPose = flipAlliance(dashboardPose);
        // }
        field2d.setRobotPose(dashboardPose);

        posePublisher.accept(dashboardPose);

        poseConsumer.accept(new TimedPose2d(getCurrentPose(), Timer.getFPGATimestamp()));
    }

    /**
     * origin is at the speaker
     * @return
     */
    public Transform2d getSpeakerTransform() {
        Alliance allianceColour = Alliance.Red;
        try {
            allianceColour = DriverStation.getAlliance().get();
        } catch (Exception e) {
            // System.out.println("no alliance colour");
        }

        if (allianceColour == Alliance.Red) {
            // Find the transform from robot to speaker when red
            return poseEstimator.getEstimatedPosition().minus(FieldConstants.poseRedSpeaker);
        }
        return poseEstimator.getEstimatedPosition().minus(FieldConstants.poseBlueSpeaker);
        // Call this method periodically when needed to aim shooter
    }

    public Transform2d getSpeakerTransformWithAlliance(Alliance alliance) {
        if (alliance == Alliance.Red) {
            // Find the transform from robot to speaker when red
            return poseEstimator.getEstimatedPosition().minus(FieldConstants.poseRedSpeaker);
        }
        return poseEstimator.getEstimatedPosition().minus(FieldConstants.poseBlueSpeaker);
        // Call this method periodically when needed to aim shooter
    }

    public Rotation2d getAngleFromSpeaker() {
        var relativePose = getSpeakerTransform();
        var desiredAngle = Math.atan2(relativePose.getY(), relativePose.getX());

        return Rotation2d.fromRadians(desiredAngle);
    }

    public Rotation2d getRotation2d() {
        return getCurrentPose().getRotation();
    }

    public boolean isAligned() {
        return MathUtil.isNear(getAngleFromSpeaker().getRadians(), Math.IEEEremainder(getCurrentPose().getRotation().getRadians(), Math.PI), Units.degreesToRadians(3));

    }

    private String getFormattedTransform() {
        var transform = getSpeakerTransform();
        return String.format("(%.3f, %.3f) %.2f degrees", transform.getX(), transform.getY(), transform.getRotation().getDegrees());
    }

    private String getFomattedPose() {
        var pose = getCurrentPose();
        return String.format("(%.3f, %.3f) %.2f degrees",
                pose.getX(),
                pose.getY(),
                pose.getRotation().getDegrees());
    }

    public Pose2d getCurrentPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets the current pose to the specified pose. This should ONLY be called
     * when the robot's position on the field is known, like at the beginning of
     * a match.
     *
     * @param newPose new pose
     */
    public void setCurrentPose(Pose2d newPose) {
        poseEstimator.resetPosition(rotationSupplier.get(), modulePositionSupplier.get(), newPose);
    }

    /**
     * Resets the position on the field to 0,0 0-degrees, with forward being
     * downfield. This resets
     * what "forward" is for field oriented driving.
     */
    public void resetFieldPosition() {
        setCurrentPose(new Pose2d());
    }

    /**
     * Transforms a pose to the opposite alliance's coordinate system. (0,0) is
     * always on the right corner of your
     * alliance wall, so for 2023, the field elements are at different coordinates
     * for each alliance.
     *
     * @param poseToFlip pose to transform to the other alliance
     * @return pose relative to the other alliance's coordinate system
     */
    private Pose2d flipAlliance(Pose2d poseToFlip) {
        return poseToFlip.relativeTo(new Pose2d(
                new Translation2d(FieldConstants.fieldWidthMeters, FieldConstants.fieldWidthMeters),
                new Rotation2d(Math.PI)));
    }

    // public void addTrajectory(PathPlannerTrajectory traj) {
    //   field2d.getObject("Trajectory").setTrajectory(traj);
    // }

    // public void resetPoseRating() {
    // xValues.clear();
    // yValues.clear();
    // }

    private Matrix<N3, N1> confidenceCalculator(EstimatedRobotPose estimation) {
        double smallestDistance = Double.POSITIVE_INFINITY;
        for (var target : estimation.targetsUsed) {
            var t3d = target.getBestCameraToTarget();
            var distance = Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2) + Math.pow(t3d.getZ(), 2));
            if (distance < smallestDistance)
                smallestDistance = distance;
        }
        double poseAmbiguityFactor = estimation.targetsUsed.size() != 1
                ? 1
                : Math.max(
                1,
                (estimation.targetsUsed.get(0).getPoseAmbiguity()
                        + CameraConstants.POSE_AMBIGUITY_SHIFTER)
                        * CameraConstants.POSE_AMBIGUITY_MULTIPLIER);
        double confidenceMultiplier = Math.max(
                1,
                (Math.max(
                        1,
                        Math.max(0, smallestDistance - CameraConstants.NOISY_DISTANCE_METERS)
                                * CameraConstants.DISTANCE_WEIGHT)
                        * poseAmbiguityFactor)
                        / (1
                        + ((estimation.targetsUsed.size() - 1) * CameraConstants.TAG_PRESENCE_WEIGHT)));

        return CameraConstants.VISION_MEASUREMENT_STANDARD_DEVIATIONS.times(confidenceMultiplier);
    }

    public void estimatorChecker(PhotonRunnable estamator) {
        var cameraPose = estamator.grabLatestEstimatedPose();
        if (cameraPose != null) {
            // New pose from vision
            var pose2d = cameraPose.estimatedPose.toPose2d();
            if (originPosition == kRedAllianceWallRightSide) {
                pose2d = flipAlliance(pose2d);
            }

            poseEstimator.addVisionMeasurement(pose2d, cameraPose.timestampSeconds,
                    confidenceCalculator(cameraPose));
        }
    }


}