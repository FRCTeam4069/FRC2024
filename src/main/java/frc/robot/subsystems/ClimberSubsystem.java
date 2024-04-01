/*
 * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/introduction.html
 * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/creating-routine.html
 * 
 * look at this if having problem with logger:
 * https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/sysid/subsystems/Shooter.java
 */


package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Direction;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.DeviceIDs;

public class ClimberSubsystem extends SubsystemBase {
    CANSparkMax climber;

    SysIdRoutine routine;

    // need to tune these !!!
    private static double pK = 0.1;
    private static double pG = 0.1;

    public ClimberSubsystem() {
        climber = new CANSparkMax(25, MotorType.kBrushless);

        routine = new SysIdRoutine(
            new SysIdRoutine.Config(), 
            new SysIdRoutine.Mechanism(this::setClimber, this::logMotor, this));

        climber.getEncoder().setPosition(0);
        climber.setSoftLimit(SoftLimitDirection.kReverse, 0);
        climber.setSoftLimit(SoftLimitDirection.kForward, 140);
        climber.setIdleMode(IdleMode.kBrake);

        climber.enableSoftLimit(SoftLimitDirection.kReverse, true);
        climber.enableSoftLimit(SoftLimitDirection.kForward, true);
        climber.setSmartCurrentLimit(40);
        climber.burnFlash();

        //climber.setSmartCurrentLimit(60);
        
    }

    public void raiseClimber() {
        climber.set(0.5);
    }

    public int positionDifference() {
        // need to do something about this
        return 1;
    }

    // a.k.a raise robot
    public void pullUp() {
        // may need to change this later
        climber.set(-pG + -pK * positionDifference());
    }

    public void setClimber(Measure<Voltage> x) {
        climber.setVoltage(x.in(Volts));
    }

    public void logMotor(SysIdRoutineLog x) {
        
    }

    // Methods for SysId: BE CAREFUL WHEN USING!!!
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }

    public void setPower(double speed){
        climber.set(-speed);
    }

    public double getEncoder(){
        return climber.getEncoder().getPosition();
    }
    public double getPower(){
        return climber.getAppliedOutput();
    }
    public double getCurrent(){
        return climber.getOutputCurrent();
    }

    public void spin(double speed) {
        if (climber.getEncoder().getPosition() > 134 && speed > 0) {
            climber.stopMotor();
        } else if (climber.getEncoder().getPosition() < 0 && speed < 0) {
            climber.stopMotor();
        } else {
            climber.set(speed);
        }
    }

    public Command setPower(edu.wpi.first.wpilibj.Relay.Direction d){
        if (d == edu.wpi.first.wpilibj.Relay.Direction.kForward){
            return this.runOnce(() -> climber.set(1));
        }
        else if(d == edu.wpi.first.wpilibj.Relay.Direction.kReverse){
            return this.runOnce(() -> climber.set(-1));
        }
        return this.runOnce(() -> climber.set(0));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("climber position", getEncoder());
    }
    
}




