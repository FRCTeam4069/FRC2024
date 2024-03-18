package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;

public class TimedPose2d {
    public Pose2d pose;
    public double timestamp;

    public TimedPose2d(Pose2d pose, double timestamp) {
        this.pose = pose;
        this.timestamp = timestamp;

    }

    public Pose2d getPose() {
        return pose;
    }

    public double getTimestamp() {
        return timestamp;
    }
    
}
