package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class CameraConstants {
    
    public static final String fCamName = "frontCamera", bCamName = "backCamera";
    public static final Transform3d robotToFrontCam = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0,0.488692,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    public static final Transform3d robotToBackCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.

}
