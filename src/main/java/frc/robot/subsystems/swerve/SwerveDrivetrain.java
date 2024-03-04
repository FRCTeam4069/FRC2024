package frc.robot.subsystems.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import javax.lang.model.util.ElementScanner14;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Millisecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.DeviceIDs;
import frc.robot.constants.DrivebaseConstants;

public class SwerveDrivetrain extends SubsystemBase {
    public SwerveModule fl, fr, bl, br;
    public SwerveDrivePoseEstimator odometry;
    public SwerveDriveKinematics kinematics = DrivebaseConstants.kinematics;
    public Pigeon2 gyro;

    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
    private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));
    private final MutableMeasure<Angle> m_angle = mutable(Degrees.of(0));
    private final MutableMeasure<Velocity<Angle>> m_angularVelocity = mutable(DegreesPerSecond.of(0));

    private Measure<Time> m_timeout = Millisecond.of(6000);
    private Measure<Velocity<Voltage>> rampRate = Volts.per(Second).of(2);

    private final StructArrayPublisher<SwerveModuleState> publisher;
    private final StructArrayPublisher<SwerveModuleState> desiredStatesPublisher;
    //private final DoublePublisher headingPublisher;

    private SwerveModuleState[] desiredStates = new SwerveModuleState[4];

    private SlewRateLimiter xSlewRateLimiter = new SlewRateLimiter(DrivebaseConstants.rampRate, -10000000000.0, 0.0);
    private SlewRateLimiter ySlewRateLimiter = new SlewRateLimiter(DrivebaseConstants.rampRate, -10000000000.0, 0.0);
    private SlewRateLimiter wSlewRateLimiter = new SlewRateLimiter(DrivebaseConstants.headingRampRate, -100000000000.0, 0.0);

    private double speedMultiplier = 1;

    private final SysIdRoutine driveRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            rampRate, null, m_timeout
        ), 
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> volts) -> {
                fl.setVoltage(volts.in(Volts));
                fr.setVoltage(volts.in(Volts));
                bl.setVoltage(volts.in(Volts));
                br.setVoltage(volts.in(Volts));
            }, 
            log -> {
                log.motor("frontLeft")
                    .voltage(m_appliedVoltage.mut_replace(
                        fl.getDriveSpeed() * RobotController.getBatteryVoltage(), Volts
                    ))
                    .linearPosition(m_distance.mut_replace(fl.getDrivePosition(), Meters))
                    .linearVelocity(m_velocity.mut_replace(fl.getDriveVelocity(), MetersPerSecond));
                log.motor("frontRight")
                    .voltage(m_appliedVoltage.mut_replace(
                        fr.getDriveSpeed() * RobotController.getBatteryVoltage(), Volts
                    ))
                    .linearPosition(m_distance.mut_replace(fr.getDrivePosition(), Meters))
                    .linearVelocity(m_velocity.mut_replace(fr.getDriveVelocity(), MetersPerSecond));
                log.motor("backLeft")
                    .voltage(m_appliedVoltage.mut_replace(
                        bl.getDriveSpeed() * RobotController.getBatteryVoltage(), Volts
                    ))
                    .linearPosition(m_distance.mut_replace(bl.getDrivePosition(), Meters))
                    .linearVelocity(m_velocity.mut_replace(bl.getDriveVelocity(), MetersPerSecond));
                log.motor("backRight")
                    .voltage(m_appliedVoltage.mut_replace(
                        br.getDriveSpeed() * RobotController.getBatteryVoltage(), Volts
                    ))
                    .linearPosition(m_distance.mut_replace(br.getDrivePosition(), Meters))
                    .linearVelocity(m_velocity.mut_replace(br.getDriveVelocity(), MetersPerSecond));
            }, this));
        

    private final SysIdRoutine steerRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null, null, null
        ), 
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> volts) -> {
                fl.setSteerVoltage(volts.in(Volts));
                fr.setSteerVoltage(volts.in(Volts));
                bl.setSteerVoltage(volts.in(Volts));
                br.setSteerVoltage(volts.in(Volts));
            }, 
            log -> {
                log.motor("frontLeft")
                    .voltage(m_appliedVoltage.mut_replace(
                        fl.getSteerSpeed() * RobotController.getBatteryVoltage(), Volts
                    ))
                    .angularPosition(m_angle.mut_replace(fl.getRadiansNow(), Radians))
                    .angularVelocity(m_angularVelocity.mut_replace(fl.getEncoderVelocity(), RadiansPerSecond));
                log.motor("frontRight")
                    .voltage(m_appliedVoltage.mut_replace(
                        fr.getSteerSpeed() * RobotController.getBatteryVoltage(), Volts
                    ))
                    .angularPosition(m_angle.mut_replace(fr.getRadiansNow(), Radians))
                    .angularVelocity(m_angularVelocity.mut_replace(fr.getEncoderVelocity(), RadiansPerSecond));
                log.motor("backLeft")
                    .voltage(m_appliedVoltage.mut_replace(
                        bl.getSteerSpeed() * RobotController.getBatteryVoltage(), Volts
                    ))
                    .angularPosition(m_angle.mut_replace(bl.getRadiansNow(), Radians))
                    .angularVelocity(m_angularVelocity.mut_replace(bl.getEncoderVelocity(), RadiansPerSecond));
                log.motor("backRight")
                    .voltage(m_appliedVoltage.mut_replace(
                        br.getSteerSpeed() * RobotController.getBatteryVoltage(), Volts
                    ))
                    .angularPosition(m_angle.mut_replace(br.getRadiansNow(), Radians))
                    .angularVelocity(m_angularVelocity.mut_replace(br.getEncoderVelocity(), RadiansPerSecond));
            }, this));

    public SwerveDrivetrain() {
        fl = new SwerveModule(
            DeviceIDs.DRIVE_FL, false, 
            DeviceIDs.STEER_FL, true, 
            DeviceIDs.ENCODER_FL, -10.547/360, true, 
            DrivebaseConstants.FRONT_LEFT);
        
        fr = new SwerveModule(
            DeviceIDs.DRIVE_FR, false, 
            DeviceIDs.STEER_FR, true, 
            DeviceIDs.ENCODER_FR, -329.854/360, true, 
            DrivebaseConstants.FRONT_RIGHT);

        bl = new SwerveModule(
            DeviceIDs.DRIVE_BL, false, 
            DeviceIDs.STEER_BL, true, 
            DeviceIDs.ENCODER_BL, -25.488/360, true, 
            DrivebaseConstants.BACK_LEFT);

        br = new SwerveModule(
            DeviceIDs.DRIVE_BR, false, 
            DeviceIDs.STEER_BR, true, 
            DeviceIDs.ENCODER_BR, -62.621/360, true, 
            DrivebaseConstants.BACK_RIGHT);

        gyro = new Pigeon2(0, "rio");
        gyro.getConfigurator().apply(DrivebaseConstants.gyroConfig);
        
        odometry = new SwerveDrivePoseEstimator(
            kinematics, 
            new Rotation2d(getRadians()), 
            getModulePositions(),
            new Pose2d());


        setPose(new Pose2d(0,0,Rotation2d.fromRadians(2*Math.PI)));

        publisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("/SwerveStates/currentStates", SwerveModuleState.struct).publish();
        
        desiredStatesPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("/SwerveStates/desiredStates", SwerveModuleState.struct).publish();

        
        startPathPlanner();
    }

    public void startPathPlanner() {
        AutoBuilder.configureHolonomic(
            this::getPose, 
            this::setPose, 
            this::getRobotRelativeSpeeds, 
            this::drive, 
            new HolonomicPathFollowerConfig(
                //translation
                new PIDConstants(0.0, 0.0, 0.0), 
                //rotation
                new PIDConstants(0.0, 0.0, 0.0), 
                DrivebaseConstants.maxVelocity,
                DrivebaseConstants.drivebaseRadius,
                new ReplanningConfig()
            ), 
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            }, 
            this);
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            fl.getPosition(),
            fr.getPosition(),
            bl.getPosition(),
            br.getPosition(),
        };
    }

    public void setModuleStates(SwerveModuleState[] moduleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DrivebaseConstants.maxVelocity);
        fl.setDesiredState(moduleStates[0]);
        fr.setDesiredState(moduleStates[1]);
        bl.setDesiredState(moduleStates[2]);
        br.setDesiredState(moduleStates[3]);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] moduleStates = {
            fl.getCurrentState(),
            fr.getCurrentState(),
            bl.getCurrentState(),
            br.getCurrentState()
        };
        return moduleStates;

    }

    /**
     * 
     * @param speeds
     */
    public void drive(ChassisSpeeds speeds) {
        desiredStates = kinematics.toSwerveModuleStates(speeds);
        setModuleStates(desiredStates);
    }

    public void fieldOrientedDrive(ChassisSpeeds speeds) {
        drive(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, new Rotation2d(getRadians())));
    }

    public Command driveCommand(DoubleSupplier xVelocity, DoubleSupplier yVelocity, DoubleSupplier angleVelocity) {
        return run(() -> {
            fieldOrientedDrive(new ChassisSpeeds(Math.pow(xVelocity.getAsDouble(), 3) * DrivebaseConstants.maxVelocity,
            Math.pow(yVelocity.getAsDouble(), 3) * DrivebaseConstants.maxVelocity,
            Math.pow(angleVelocity.getAsDouble(), 3) * DrivebaseConstants.maxAngularVelocity));
        });
    }

    public Command teleopDriveCommand(DoubleSupplier xVelocity, DoubleSupplier yVelocity, DoubleSupplier angleVelocity, BooleanSupplier halfSpeed) {
        if (Math.abs(xVelocity.getAsDouble()) < 0.01 && Math.abs(yVelocity.getAsDouble()) < 0.01 && Math.abs(angleVelocity.getAsDouble()) < 0.01) {
            speedMultiplier = 0;
        } else if (halfSpeed.getAsBoolean()) {
            speedMultiplier = 0.1;
        } else {
            speedMultiplier = 1;
        }
        return run(() -> {
            fieldOrientedDrive(new ChassisSpeeds(
                (Math.pow(xVelocity.getAsDouble(), 3)*speedMultiplier * DrivebaseConstants.maxVelocity),
                (Math.pow(yVelocity.getAsDouble(), 3)*speedMultiplier * DrivebaseConstants.maxVelocity),
                (Math.pow(angleVelocity.getAsDouble(), 3)*speedMultiplier * DrivebaseConstants.maxAngularVelocity)));
        });
    }

    public Pose2d getPose() {
        return odometry.getEstimatedPosition();
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public void setPose(Pose2d pose) {
        resetGyro();
        odometry.resetPosition(new Rotation2d(getRadians()), getModulePositions(), pose);
    }

    public void addVisionMeasurement(Pose2d poseMeters, double timstampSeconds) {
        var deltaX = poseMeters.getX() - getPose().getX();
        var deltaY = poseMeters.getY() - getPose().getY();
        var delta = Math.hypot(deltaX, deltaY);
        if (delta > 1.0) {
            odometry.addVisionMeasurement(poseMeters, timstampSeconds);
        }

    }

    public void resetPose() {
        setPose(new Pose2d());
    }

    @Override
    public void periodic() {
        fl.update();
        fr.update();
        bl.update();
        br.update();

        odometry.update(new Rotation2d(getRadians()), getModulePositions());

        var pose = getPose();

        publisher.set(getModuleStates());
        desiredStatesPublisher.set(desiredStates);

        try {
            var headings = getModuleHeadings();
            var steerHeadings = getModuleSteerHeadings();

            SmartDashboard.putNumber("robot heading", getDegrees());
            SmartDashboard.putNumber("voltage", RobotController.getBatteryVoltage());
            SmartDashboard.putNumber("fl get", fl.getDriveSpeed());

            SmartDashboard.putNumber("FL drive pos", fl.getDrivePosition());
            SmartDashboard.putNumber("FR drive pos", fr.getDrivePosition());
            SmartDashboard.putNumber("BL drive pos", bl.getDrivePosition());
            SmartDashboard.putNumber("BR drive pos", br.getDrivePosition());

            // SmartDashboard.putNumber("FL current mps", fl.getDriveVelocity());
            // SmartDashboard.putNumber("FR current mps", fr.getDriveVelocity());
            // SmartDashboard.putNumber("BL current mps", bl.getDriveVelocity());
            // SmartDashboard.putNumber("BR current mps", br.getDriveVelocity());

            // SmartDashboard.putNumber("FL steer heading", steerHeadings[0]);
            // SmartDashboard.putNumber("FR steer heading", steerHeadings[1]);
            // SmartDashboard.putNumber("BL steer heading", steerHeadings[2]);
            // SmartDashboard.putNumber("BR steer heading", steerHeadings[3]);

            // SmartDashboard.putNumber("FL absolute heading", headings[0]);
            // SmartDashboard.putNumber("FR absolute heading", headings[1]);
            // SmartDashboard.putNumber("BL absolute heading", headings[2]);
            // SmartDashboard.putNumber("BR absolute heading", headings[3]);

            // SmartDashboard.putNumber("FL desired mps", fl.getDesiredState().speedMetersPerSecond);
            // SmartDashboard.putNumber("FR desired mps", fr.getDesiredState().speedMetersPerSecond);
            // SmartDashboard.putNumber("BL desired mps", bl.getDesiredState().speedMetersPerSecond);
            // SmartDashboard.putNumber("BR desired mps", br.getDesiredState().speedMetersPerSecond);

            // SmartDashboard.putNumber("FL desired angle", fl.getDesiredState().angle.getDegrees());
            // SmartDashboard.putNumber("FR desired angle", fr.getDesiredState().angle.getDegrees());
            // SmartDashboard.putNumber("BL desired angle", bl.getDesiredState().angle.getDegrees());
            // SmartDashboard.putNumber("BR desired angle", br.getDesiredState().angle.getDegrees());

            SmartDashboard.putNumber("x", pose.getX());
            SmartDashboard.putNumber("y", pose.getY());
            SmartDashboard.putNumber("degrees", pose.getRotation().getDegrees());
        } catch (Exception e) {
            SmartDashboard.putString("AAAAAAA", "WHY GOD WHY");
        }
    }

    public double[] getModuleHeadings() {
        double[] headings = {fl.getRadians(), fr.getRadians(), bl.getRadians(), br.getRadians()};
        return headings;
    }

    public double[] getModuleSteerHeadings() {
        double[] headings = {fl.getSteerPosition(), fr.getSteerPosition(), bl.getSteerPosition(), br.getSteerPosition()};
        return headings;
    }

    public void stopModules() {
        fl.stop();
        fr.stop();
        bl.stop();
        br.stop();
    }

    public Command angleModulesCommand(DoubleSupplier y, DoubleSupplier x) {
        return run(() -> angleModules(Rotation2d.fromRadians(Math.atan2(y.getAsDouble(), x.getAsDouble()))));
    }

    public void angleModules(Rotation2d angle) {
        SmartDashboard.putNumber("input angle", angle.getDegrees());
        fl.setDesiredState(new SwerveModuleState(0.00, angle));
        fr.setDesiredState(new SwerveModuleState(0.00, angle));
        bl.setDesiredState(new SwerveModuleState(0.00, angle));
        br.setDesiredState(new SwerveModuleState(0.00, angle));
        SwerveModuleState help[] = {
            new SwerveModuleState(0.0, angle),
            new SwerveModuleState(0.0, angle),
            new SwerveModuleState(0.0, angle),
            new SwerveModuleState(0.0, angle)
        };
        desiredStates = help;
    }

    public void resetGyro() {
        setRadians(2*Math.PI);
    }

    public Command resetGyroCommand() {
        return runOnce(() -> resetGyro());
    }

    public StatusCode setRadians(double rads) {
        return gyro.setYaw(Units.radiansToDegrees(rads));
    }
    public double getDegrees() {
        return gyro.getYaw().getValueAsDouble();
    }

    public double getRadians() {
        return Units.degreesToRadians(getDegrees());
    }

    public void coast() {
        fl.stop();
        fr.stop();
        bl.stop();
        br.stop();
    }

    public void setVoltage(double volts) {
        fl.stopSteerMotor();
        fr.stopSteerMotor();
        bl.stopSteerMotor();
        br.stopSteerMotor();

        fl.setVoltage(volts);
        fr.setVoltage(volts);
        bl.setVoltage(volts);
        br.setVoltage(volts);

    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return driveRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return driveRoutine.dynamic(direction);
    }

    public final double delay = 3;
    public final double timeout = 10;

    public Command sysIdSteerTest() {
        return steerRoutine
            .quasistatic(SysIdRoutine.Direction.kForward)
            .withTimeout(20.0)
            .andThen(Commands.waitSeconds(delay))
            .andThen(
                steerRoutine.quasistatic(SysIdRoutine.Direction.kReverse).withTimeout(20.0))
            .andThen(Commands.waitSeconds(delay))
            .andThen(steerRoutine.dynamic(SysIdRoutine.Direction.kForward).withTimeout(20.0))
            .andThen(Commands.waitSeconds(delay))
            .andThen(steerRoutine.dynamic(SysIdRoutine.Direction.kReverse).withTimeout(20.0));
    }

    public Command sysIdDriveTest() {
        return driveRoutine
            .quasistatic(SysIdRoutine.Direction.kForward)
            .withTimeout(timeout)
            .andThen(Commands.waitSeconds(delay))
            .andThen(
                driveRoutine.quasistatic(SysIdRoutine.Direction.kReverse).withTimeout(timeout))
            .andThen(Commands.waitSeconds(delay))
            .andThen(driveRoutine.dynamic(SysIdRoutine.Direction.kForward).withTimeout(timeout))
            .andThen(Commands.waitSeconds(delay))
            .andThen(driveRoutine.dynamic(SysIdRoutine.Direction.kReverse).withTimeout(timeout));
    }

    public Command sysIdDriveTestQuasistatic() {
        return driveRoutine
            .quasistatic(SysIdRoutine.Direction.kForward)
            .withTimeout(timeout)
            .andThen(Commands.waitSeconds(delay))
            .andThen(
                driveRoutine.quasistatic(SysIdRoutine.Direction.kReverse).withTimeout(timeout));
    }

    public Command sysIdDriveTestDynamic() {
        return driveRoutine
            .dynamic(SysIdRoutine.Direction.kForward).withTimeout(timeout)
            .andThen(Commands.waitSeconds(delay))
            .andThen(driveRoutine.dynamic(SysIdRoutine.Direction.kReverse).withTimeout(timeout));
    }

}
