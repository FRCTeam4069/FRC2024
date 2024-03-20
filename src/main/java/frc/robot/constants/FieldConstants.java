package frc.robot.constants;

import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldConstants {

    public static final double fieldLengthMeters = Units.inchesToMeters(651.25);
    public static final double fieldWidthMeters = Units.inchesToMeters(315.5);

    public static final Pose2d poseRedSpeaker = new Pose2d(new Translation2d(-0.038099999999999995, 2.465832), new Rotation2d(Units.degreesToRadians(0))); //x equals total field length minus abs value of red x, y equals total field width minus blue y
    public static final Pose2d poseBlueSpeaker = new Pose2d(new Translation2d(-0.038099999999999995, 5.10), new Rotation2d(Units.degreesToRadians(0)));

    public static final Pose2d poseRedAutoLeft = new Pose2d(new Translation2d(), new Rotation2d());
    public static final Pose2d poseRedAutoCenter = new Pose2d(new Translation2d(), new Rotation2d());
    public static final Pose2d poseRedAutoRight = new Pose2d(new Translation2d(), new Rotation2d());
    public static final Pose2d poseBlueAutoLeft = new Pose2d(new Translation2d(), new Rotation2d());
    public static final Pose2d poseBlueAutoCenter = new Pose2d(new Translation2d(), new Rotation2d());
    public static final Pose2d poseBlueAutoRight = new Pose2d(new Translation2d(), new Rotation2d());
    //make pose2d for positions where the robot starts in auto to compare the estimated pose to

  public static Alliance alliance = DriverStation.getAlliance().get();

  public static final Pose2d blueTrap = new Pose2d(new Translation2d(4.2, 5.5), new Rotation2d(122));
  public static final Pose2d redTrap = new Pose2d(new Translation2d(1, 1), new Rotation2d(1));

}
