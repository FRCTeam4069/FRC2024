package frc.robot.subsystems.Limelight;

public class CALM {// Computer Automated Localization Mechanism
    CameraIsAsCameraDoes rf, rr, lf, ll;

    public CALM(String rf, String rr, String lf, String ll){
        this.rf = new CameraIsAsCameraDoes(rf);
        this.rr = new CameraIsAsCameraDoes(rr);
        this.lf = new CameraIsAsCameraDoes(lf);
        this.ll = new CameraIsAsCameraDoes(ll);
    }
    
    double estX = 0;
    double estY = 0;
    double estZ = 0;

    

    public void getRobotPose(){
        estX = (rf.getRobotPose2d().getX() + rr.getRobotPose2d().getX() + lf.getRobotPose2d().getX() + ll.getRobotPose2d().getX()) / 4;
        estY = (rf.getRobotPose2d().getX() + rr.getRobotPose2d().getX() + lf.getRobotPose2d().getX() + ll.getRobotPose2d().getX()) / 4;

    }

    public double getX(){
        return estX;
    }
    public double getY(){
        return estY;
    }

    public double getDistanceToApriltag(){
        return Math.hypot((lf.getXDistanceToApriltag(7,2) + rf.getXDistanceToApriltag(7, 2)) / 2, ((lf.getYDistanceToApriltag(7, 2) + rf.getYDistanceToApriltag(7, 2)) / 2));
    }

}
