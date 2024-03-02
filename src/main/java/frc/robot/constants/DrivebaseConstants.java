package frc.robot.constants;

import com.ctre.phoenix6.configs.Pigeon2Configuration;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class DrivebaseConstants {
    public static final double driveConversionFactor = 0.06209840731609397;
    public static final double steerConversionFactor = 16.8;
    public static final double moduleOffset = Units.inchesToMeters(10.375);
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d(moduleOffset, moduleOffset), //fl
        new Translation2d(moduleOffset, -moduleOffset), //fr
        new Translation2d(-moduleOffset, moduleOffset), //bl
        new Translation2d(-moduleOffset, -moduleOffset) //br
    );
    public static final Pigeon2Configuration gyroConfig = new Pigeon2Configuration();
    public static final double maxVelocity = Units.feetToMeters(22.0);
    public static final double maxAngularVelocity =
        maxVelocity / new Rotation2d(moduleOffset, moduleOffset).getRadians();
    public static final double KS = 0.01;
    public static final double KV = 0.15;
    public static final double KP = 0.45;

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
        0.28903,
        1.7657,
        0.31505,

        KP,
        0.0,
        0.0
    );

    public static ModuleCoefficient BACK_RIGHT = new ModuleCoefficient (
        0.1647, //-0.21156, 
        1.7998,
        0.33328, //0.65603,

        KP,
        0.0,
        0.0
    );
    public static ModuleCoefficient FRONT_LEFT = new ModuleCoefficient (
        0.18494, // -0.25525,
        1.7955,
        0.25813, // 0.60935,

        KP,
        0.0,
        0.0
    );
    public static ModuleCoefficient FRONT_RIGHT = new ModuleCoefficient (
        0.23188,
        1.7732,
        0.34659,

        KP,
        0.0,
        0.0
    );

    public static final SimpleMotorFeedforward BACK_LEFT_FEEDFORWARD = new SimpleMotorFeedforward(BACK_LEFT.kS, BACK_LEFT.kV, BACK_LEFT.kA);
    public static final SimpleMotorFeedforward BACK_RIGHT_FEEDFORWARD = new SimpleMotorFeedforward(BACK_RIGHT.kS, BACK_RIGHT.kV, BACK_RIGHT.kA);
    public static final SimpleMotorFeedforward FRONT_LEFT_FEEDFORWARD = new SimpleMotorFeedforward(FRONT_LEFT.kS, FRONT_LEFT.kV, FRONT_LEFT.kA);
    public static final SimpleMotorFeedforward FRONT_RIGHT_FEEDFORWARD = new SimpleMotorFeedforward(FRONT_RIGHT.kS, FRONT_RIGHT.kV, FRONT_RIGHT.kA);
}