package frc.robot.subsystems.swerve;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Millisecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecond;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.RobotContainer;
import frc.robot.constants.DeviceIDs;
import frc.robot.constants.DrivebaseConstants;

public class SwerveDrivetrain extends SubsystemBase {
    public SwerveModule fl, fr, bl, br;
    public SwerveDriveOdometry odometry;
    public SwerveDriveKinematics kinematics = DrivebaseConstants.kinematics;
    public Pigeon2 gyro;

    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
    private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

    private Measure<Time> m_timeout = Millisecond.of(6000);
    private Measure<Velocity<Voltage>> rampRate = Volts.per(Second).of(2);

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
            rampRate, null, m_timeout
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
                    .linearPosition(m_distance.mut_replace(fl.getDrivePosition(), Meters))
                    .linearVelocity(m_velocity.mut_replace(fl.getDriveVelocity(), MetersPerSecond));
                log.motor("frontRight")
                    .voltage(m_appliedVoltage.mut_replace(
                        fr.getSteerSpeed() * RobotController.getBatteryVoltage(), Volts
                    ))
                    .linearPosition(m_distance.mut_replace(fr.getSteerPosition(), Meters))
                    .linearVelocity(m_velocity.mut_replace(fr.getDriveVelocity(), MetersPerSecond));
                log.motor("backLeft")
                    .voltage(m_appliedVoltage.mut_replace(
                        bl.getSteerSpeed() * RobotController.getBatteryVoltage(), Volts
                    ))
                    .linearPosition(m_distance.mut_replace(bl.getDrivePosition(), Meters))
                    .linearVelocity(m_velocity.mut_replace(bl.getDriveVelocity(), MetersPerSecond));
                log.motor("backRight")
                    .voltage(m_appliedVoltage.mut_replace(
                        br.getSteerSpeed() * RobotController.getBatteryVoltage(), Volts
                    ))
                    .linearPosition(m_distance.mut_replace(br.getDrivePosition(), Meters))
                    .linearVelocity(m_velocity.mut_replace(br.getDriveVelocity(), MetersPerSecond));
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
        
        odometry = new SwerveDriveOdometry(
            kinematics, 
            new Rotation2d(getRadians()), 
            getModulePositions());

        setPose(new Pose2d());

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

    /**
     * 
     * @param speeds
     */
    public void drive(ChassisSpeeds speeds) {
        setModuleStates(kinematics.toSwerveModuleStates(speeds));
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

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        resetGyro();
        odometry.resetPosition(new Rotation2d(getRadians()), getModulePositions(), pose);
    }

    @Override
    public void periodic() {

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

            SmartDashboard.putNumber("FL current mps", fl.getDriveVelocity());
            SmartDashboard.putNumber("FR current mps", fr.getDriveVelocity());
            SmartDashboard.putNumber("BL current mps", bl.getDriveVelocity());
            SmartDashboard.putNumber("BR current mps", br.getDriveVelocity());

            SmartDashboard.putNumber("FL steer heading", steerHeadings[0]);
            SmartDashboard.putNumber("FR steer heading", steerHeadings[1]);
            SmartDashboard.putNumber("BL steer heading", steerHeadings[2]);
            SmartDashboard.putNumber("BR steer heading", steerHeadings[3]);

            SmartDashboard.putNumber("FL absolute heading", headings[0]);
            SmartDashboard.putNumber("FR absolute heading", headings[1]);
            SmartDashboard.putNumber("BL absolute heading", headings[2]);
            SmartDashboard.putNumber("BR absolute heading", headings[3]);

            SmartDashboard.putNumber("FL desired mps", fl.getDesiredState().speedMetersPerSecond);
            SmartDashboard.putNumber("FR desired mps", fr.getDesiredState().speedMetersPerSecond);
            SmartDashboard.putNumber("BL desired mps", bl.getDesiredState().speedMetersPerSecond);
            SmartDashboard.putNumber("BR desired mps", br.getDesiredState().speedMetersPerSecond);

            SmartDashboard.putNumber("FL desired angle", fl.getDesiredState().angle.getDegrees());
            SmartDashboard.putNumber("FR desired angle", fr.getDesiredState().angle.getDegrees());
            SmartDashboard.putNumber("BL desired angle", bl.getDesiredState().angle.getDegrees());
            SmartDashboard.putNumber("BR desired angle", br.getDesiredState().angle.getDegrees());
        } catch (Exception e) {
            SmartDashboard.putString("AAAAAAA", "WHY GOD WHY");
        }
        fl.update();
        fr.update();
        bl.update();
        br.update();

        odometry.update(new Rotation2d(getRadians()), getModulePositions());
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
        fl.setDesiredState(new SwerveModuleState(0.006, angle));
        fr.setDesiredState(new SwerveModuleState(0.006, angle));
        bl.setDesiredState(new SwerveModuleState(0.006, angle));
        br.setDesiredState(new SwerveModuleState(0.006, angle));
    }

    public void resetGyro() {
        setRadians(0.0);
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
