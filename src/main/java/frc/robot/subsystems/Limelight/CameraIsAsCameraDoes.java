package frc.robot.subsystems.Limelight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraIsAsCameraDoes extends SubsystemBase{
    
String cameraName;
LimelightHelpers limeLight = new LimelightHelpers();

    public CameraIsAsCameraDoes(String camName) {
        cameraName = camName;

    }

    public void printNumbers() {
        SmartDashboard.putNumber(cameraName + " tx:" , limeLight.getTX(cameraName));
        SmartDashboard.putNumber(cameraName + " ty: ", limeLight.getTY(cameraName));
        SmartDashboard.putNumber(cameraName + " fiducial ID: ", limeLight.getFiducialID(cameraName));
    }
}
