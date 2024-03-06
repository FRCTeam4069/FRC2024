package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class BetterCameraConstants {
    //public static final double targetWidth =
            // Units.inchesToMeters(41.30) - Units.inchesToMeters(6.70); // meters

    // See
    // https://firstfrc.blob.core.windows.net/frc2020/PlayingField/2020FieldDrawing-SeasonSpecific.pdf
    // page 197
    // public static final double targetHeight =
            // Units.inchesToMeters(98.19) - Units.inchesToMeters(81.19); // meters

    // See https://firstfrc.blob.core.windows.net/frc2020/PlayingField/LayoutandMarkingDiagram.pdf
    // pages 4 and 5

    public static final Pose3d kTargetPose =
            new Pose3d(
                    new Translation3d(-0.038099999999, 5.547867999999999, 1.4511020000000001),
                    new Rotation3d(0.0, 0.0, Units.degreesToRadians(180)));


    public static Transform3d kCameraToRobot = new Transform3d(0.0, 0.0, 0.0, new Rotation3d());

}
