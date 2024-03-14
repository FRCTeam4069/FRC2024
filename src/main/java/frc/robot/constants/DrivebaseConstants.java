package frc.robot.constants;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.PubSub;

public final class DrivebaseConstants {
    public static final double driveConversionFactor = ((4.00 * Math.PI) * 0.0254) / 6.12;
    public static final double steerConversionFactor = 16.8;
    public static final double moduleOffset = Units.inchesToMeters(10.375);
    public static final double drivebaseRadius = Math.hypot(moduleOffset, moduleOffset);
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d(moduleOffset, moduleOffset), //fl
        new Translation2d(moduleOffset, -moduleOffset), //fr
        new Translation2d(-moduleOffset, moduleOffset), //bl
        new Translation2d(-moduleOffset, -moduleOffset) //br
    );
    public static final Pigeon2Configuration gyroConfig = new Pigeon2Configuration().withMountPose(new MountPoseConfigs().withMountPoseYaw(0.0));
    public static final double maxVelocity = Units.feetToMeters(19.3);
    public static final double maxAngularVelocity =
        maxVelocity / new Rotation2d(moduleOffset, moduleOffset).getRadians();
    public static final double rampRate = 0.17;
    public static final double headingRampRate = 10000.0;
    private static final double KS = 0.63253;
    private static final double KV = 2.2936;
    private static final double KA = 0.18409;
    private static final double KP = 0.40;
    private static final double KD = 0.00;

    public static final class AlignConstants {
        public static final double kP = 0.20;
        public static final double kI = 0.0;
        public static final double kD = 0.001;
        public static final double kS = 0.005;
        public static final double kV = 0.01;

        public static final double positionTolerance = Units.degreesToRadians(2);
        public static final double velocityTolerance = 0.06;
    }

    public static final class AutoAlignConstants {
        public static final double kP = 0.19;
        public static final double kI = 0.0;
        public static final double kD = 0.020;
        public static final double kS = 0.002;
        public static final double kV = 0.01;
        //public static final double powerLimit = 0.75;
        public static final double positionTolerance = Units.degreesToRadians(2);
        //public static final double velocityTolerance = 0.005;

    }

    public static class ModuleCoefficient {
        public double kS = 0.29966;
        public double kV = 1.7297;
        public double kA = 0.21697;

        public double kP = KP;
        public double kI = 0.0;
        public double kD = 0.0;

        public ModuleCoefficient(
            double kS, double kV, double kA,
            double kP, double kI, double kD
        ) {
            this.kS = kS;
            this.kV = kV;
            this.kA = kA;
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }
    }

    public static ModuleCoefficient BACK_LEFT = new ModuleCoefficient(
        KS,
        KV,
        KA,

        KP,
        0.0,
        KD
    );

    public static ModuleCoefficient BACK_RIGHT = new ModuleCoefficient (
        KS,
        KV,
        KA,

        KP,
        0.0,
        KD
    );
    public static ModuleCoefficient FRONT_LEFT = new ModuleCoefficient (
        KS,
        KV,
        KA,

        KP,
        0.0,
        KD
    );
    public static ModuleCoefficient FRONT_RIGHT = new ModuleCoefficient (
        KS,
        KV,
        KA,

        KP,
        0.0,
        KD
    );

    public static final SimpleMotorFeedforward BACK_LEFT_FEEDFORWARD = new SimpleMotorFeedforward(BACK_LEFT.kS, BACK_LEFT.kV, BACK_LEFT.kA);
    public static final SimpleMotorFeedforward BACK_RIGHT_FEEDFORWARD = new SimpleMotorFeedforward(BACK_RIGHT.kS, BACK_RIGHT.kV, BACK_RIGHT.kA);
    public static final SimpleMotorFeedforward FRONT_LEFT_FEEDFORWARD = new SimpleMotorFeedforward(FRONT_LEFT.kS, FRONT_LEFT.kV, FRONT_LEFT.kA);
    public static final SimpleMotorFeedforward FRONT_RIGHT_FEEDFORWARD = new SimpleMotorFeedforward(FRONT_RIGHT.kS, FRONT_RIGHT.kV, FRONT_RIGHT.kA);
}
