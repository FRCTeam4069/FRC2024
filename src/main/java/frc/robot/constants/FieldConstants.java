package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {

    public static final double fieldLengthMeters = Units.inchesToMeters(651.25);
    public static final double fieldWidthMeters = Units.inchesToMeters(315.5);

    public static final Pose2d poseRedSpeaker = new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(Units.degreesToRadians(0))); //x equals total field length plus abs value of red x, y equals total field width minus blue y
    public static final Pose2d poseBlueSpeaker = new Pose2d(new Translation2d(-0.038099999999999995, 5.547867999999999), new Rotation2d(Units.degreesToRadians(0)));

    public static final Pose2d poseRedAutoLeft = new Pose2d(new Translation2d(), new Rotation2d());
    public static final Pose2d poseRedAutoCenter = new Pose2d(new Translation2d(), new Rotation2d());
    public static final Pose2d poseRedAutoRight = new Pose2d(new Translation2d(), new Rotation2d());
    public static final Pose2d poseBlueAutoLeft = new Pose2d(new Translation2d(), new Rotation2d());
    public static final Pose2d poseBlueAutoCenter = new Pose2d(new Translation2d(), new Rotation2d());
    public static final Pose2d poseBlueAutoRight = new Pose2d(new Translation2d(), new Rotation2d());
    //make pose2d for positions where the robot starts in auto to compare the estimated pose to

}
