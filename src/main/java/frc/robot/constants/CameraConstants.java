package frc.robot.constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class CameraConstants {
    
    public static boolean USE_VISION = true;

    public static final double apriltagAmbiguityThreshold = 0.4;
    public static final double POSE_AMBIGUITY_SHIFTER = 0.2;
    public static final double POSE_AMBIGUITY_MULTIPLIER = 4;
    public static final double NOISY_DISTANCE_METERS = 2.5;
    public static final double DISTANCE_WEIGHT = 7;
    public static final int TAG_PRESENCE_WEIGHT = 10;

    //TODO find correct transforms
    public static final Transform3d robotCenterToFrontCam = new Transform3d(new Translation3d(Units.inchesToMeters(-11.5), Units.inchesToMeters(0), Units.inchesToMeters(6.5)), new Rotation3d(0, Units.degreesToRadians(28.0), Units.degreesToRadians(180)));
    public static final Transform3d robotCenterToRightCam = new Transform3d(new Translation3d(Units.inchesToMeters(-6.5), Units.inchesToMeters(-10.5), Units.inchesToMeters(11.5)), new Rotation3d(0, Units.degreesToRadians(27.8), Units.degreesToRadians(90)));

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
