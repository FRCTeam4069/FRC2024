package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class CameraConstants {
    
    public static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    public static boolean USE_VISION = true;

    //simon fix this later idk what your cooking here
    public static Transform3d robotToFrontCam;

    public static final double apriltagAmbiguityThreshold = 0.2;
    public static final double POSE_AMBIGUITY_SHIFTER = 0.2;
    public static final double POSE_AMBIGUITY_MULTIPLIER = 4;
    public static final double NOISY_DISTANCE_METERS = 2.5;
    public static final double DISTANCE_WEIGHT = 7;
    public static final int TAG_PRESENCE_WEIGHT = 10;

    public static final String fCamName = "frontCamera", sCamName = "sideCamera";

    public static final Transform3d robotCenterToFrontCam = new Transform3d(new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)), new Rotation3d(0,0.488692,0));
    public static final Transform3d robotCenterToSideCam = new Transform3d(new Translation3d(Units.inchesToMeters(0),Units.inchesToMeters(0),Units.inchesToMeters(0)), new Rotation3d(0,0,0));

     /**
     * Standard deviations of model states. Increase these numbers to trust your
     * model's state estimates less. This
     * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then
     * meters.
     */
    public static final Matrix<N3, N1> VISION_MEASUREMENT_STANDARD_DEVIATIONS = Matrix.mat(Nat.N3(), Nat.N1())
        .fill(
            // if these numbers are less than one, multiplying will do bad things
            1, // x
            1, // y
            1 * Math.PI // theta
        );

    /**
     * Standard deviations of the vision measurements. Increase these numbers to
     * trust global measurements from vision
     * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and
     * radians.
     */
    public static final Matrix<N3, N1> STATE_STANDARD_DEVIATIONS = Matrix.mat(Nat.N3(), Nat.N1())
        .fill(
            // if these numbers are less than one, multiplying will do bad things
            .1, // x
            .1, // y
            .1);

}
