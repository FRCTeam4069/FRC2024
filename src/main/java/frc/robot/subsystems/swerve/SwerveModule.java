package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.DrivebaseConstants;
import frc.robot.constants.DrivebaseConstants.ModuleCoefficient;

public class SwerveModule {
    private CANSparkFlex drive;
    private CANSparkMax steer;
    private CANcoder encoder;
    private PIDController steerPIDController;
    private SimpleMotorFeedforward driveFeedforward;
    private SwerveModuleState desiredStates = new SwerveModuleState();
    private double drivePower = 0.0;
    private double steerPower = 0.0;
    private double heading = 0.0;
    private boolean limitInput = true;

    /**
     * Create swerve module
     * @param driveId drive motor ID
     * @param driveInverted is drive inverted?
     * @param steerId steer motor ID
     * @param steerInverted is steer inverted?
     * @param encoderId CANCoder ID
     * @param encoderOffset CANCoder offset in rotations
     * @param encoderInversed is CANCoder inverted?
     */
    public SwerveModule(
        int driveId, 
        boolean driveInverted,
        int steerId, 
        boolean steerInverted, 
        int encoderId, 
        double encoderOffset, 
        boolean encoderInverted,
        ModuleCoefficient moduleCoefficient
        ) {

        drive = new CANSparkFlex(driveId, MotorType.kBrushless);
        steer = new CANSparkMax(steerId, MotorType.kBrushless);
        encoder = new CANcoder(encoderId, "rio");
        driveFeedforward = new SimpleMotorFeedforward(moduleCoefficient.kS, moduleCoefficient.kV, moduleCoefficient.kA);
        
        drive.getEncoder().setPosition(0);
        drive.setInverted(driveInverted);
        steer.getEncoder().setPosition(0);
        steer.setInverted(steerInverted);

        drive.setIdleMode(IdleMode.kBrake);
        steer.setIdleMode(IdleMode.kBrake);

        drive.getEncoder().setPositionConversionFactor(DrivebaseConstants.driveConversionFactor);
        drive.getEncoder().setVelocityConversionFactor(DrivebaseConstants.driveConversionFactor);
        steer.getEncoder().setPositionConversionFactor(DrivebaseConstants.steerConversionFactor);
        steer.getEncoder().setVelocityConversionFactor(DrivebaseConstants.steerConversionFactor);

        var config = new MagnetSensorConfigs();
        config.SensorDirection = encoderInverted ? SensorDirectionValue.CounterClockwise_Positive : SensorDirectionValue.Clockwise_Positive;
        config.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        config.MagnetOffset = encoderOffset;
        encoder.getConfigurator().apply(config);

        steerPIDController = new PIDController(moduleCoefficient.kP, moduleCoefficient.kI, moduleCoefficient.kD);
        steerPIDController.enableContinuousInput(0.0, Math.PI*2);

    }

    /**
     * please call this to update the encoders
     */
    public void update() {
        heading = encoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
    }

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(getDriveVelocity(), getRotation2d());
    }

    public double getRadiansNow() {
        heading = encoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
        return heading;
    }

    public double getEncoderVelocity() {
        return encoder.getVelocity().getValueAsDouble() * 2 * Math.PI;
    }



    /**
     * get heading of the module with CANCoder
     * @return between -pi and +pi
     */
    public double getRadians() {
        return heading;
    }

    public Rotation2d getRotation2d() {
        return new Rotation2d(getRadians());
    }

    /**
     * get velocity of the drive motor
     * @return m/s (hopefully) (almost certainly)
     */
    public double getDriveVelocity() {
        return drive.getEncoder().getVelocity() / 60.0;
    }

    /**
     * get angular velocity of the steer motor
     * @return deg/s (hopefully)
     */
    public double getSteerVelocity() {
        return steer.getEncoder().getVelocity() / 60.0;
    }

    /**
     * get position of the drive motor
     * @return meters (hopefully)
     */
    public double getDrivePosition() {
        return drive.getEncoder().getPosition();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getRadians()));
    }

    public void setInputLimit(boolean help) {
        limitInput = help;
    }

    public void setDesiredState(SwerveModuleState state) {
        if (limitInput && Math.abs(state.speedMetersPerSecond) < 0.01) {
            stop();
            return;
        }
        

        state = SwerveModuleState.optimize(state, getRotation2d());
        desiredStates = state;

        var newDrivePower = driveFeedforward.calculate(state.speedMetersPerSecond);
        var newSteerPower = steerPIDController.calculate(getRadians(), state.angle.getRadians());
        if (Math.abs(newDrivePower-drivePower) > 0.01) {
            drivePower = newDrivePower;
            drive.set(drivePower);
        }
        if (Math.abs(newSteerPower-steerPower) > 0.01) {
            steerPower = newSteerPower;
            steer.set(steerPower);
        }

    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), getRotation2d());
    }

    public SwerveModuleState getDesiredState() {
        return desiredStates;
    }

    /**
     * 
     * @return degrees
     */
    public double getSteerPosition() {
        return steer.getEncoder().getPosition();
    }

    public void coast() {
        stop();
        drive.setIdleMode(IdleMode.kCoast);
        steer.setIdleMode(IdleMode.kCoast);
    }

    public void brake() {
        stop();
        drive.setIdleMode(IdleMode.kBrake);
        steer.setIdleMode(IdleMode.kBrake);
    }

    public void stop() {
        drive.stopMotor();
        steer.stopMotor();
    }

    public void stopSteerMotor() {
        steer.stopMotor();
    }

    public void setVoltage(double volts) {
        drive.setVoltage(volts);
    }

    public void setSteerVoltage(double volts) {
        steer.setVoltage(volts);
    }

    /**
     * drive motor speed being applied
     * @return -1.0 to 1.0
     */
    public double getDriveSpeed() {
        return drive.getAppliedOutput();
    }

    /**
     * steer motor speed being applied
     * @return -1.0 to 1.0
     */
    public double getSteerSpeed() {
        return steer.getAppliedOutput();
    }

    /**
     * drive motor speed being applied
     * (i know this is stupid, im dont care)
     * @return -1.0 to 1.0
     */
    public double getVoltage() {
        return drive.getAppliedOutput();
    }
    
}
