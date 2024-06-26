package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.DeviceIDs;
import frc.robot.constants.IntakeConstants;

public class IntakeController extends SubsystemBase {
    CANSparkMax feedMotor, artMotor1;

    private SlewRateLimiter limit;

    private final double LOWER = IntakeConstants.LOWER_POSITION - 6, 
                         UPPER = IntakeConstants.UPPER_POSITION - 3;


    public IntakeController(){
        feedMotor = new CANSparkMax(DeviceIDs.INTAKE_FEED, MotorType.kBrushless);
        artMotor1 = new CANSparkMax(DeviceIDs.INTAKE_ARTICULATE, MotorType.kBrushless);

        feedMotor.setIdleMode(IdleMode.kCoast);
        artMotor1.setIdleMode(IdleMode.kBrake);

        artMotor1.getEncoder().setPosition(UPPER);

        artMotor1.setSoftLimit(SoftLimitDirection.kForward, 0);
        artMotor1.setSoftLimit(SoftLimitDirection.kReverse, IntakeConstants.UPPER_POSITION);
        
        limit = new SlewRateLimiter(.94);


        artMotor1.setSmartCurrentLimit(20);
        feedMotor.setSmartCurrentLimit(40);
        artMotor1.burnFlash();
        // feedMotor.burnFlash();
    }

    public void driveFeed(){
        feedMotor.set((-0.85));
    }
    public void backIntake(){
        feedMotor.set(limit.calculate(1));
    }
    public void stopFeed(){
        feedMotor.stopMotor();
    }
    public void driveArt(double speed){
        artMotor1.set(speed);
        //System.out.println("encoder" + getEncoder());
    }
    public void stopArt(){
        artMotor1.stopMotor();
    }

    public double getEncoder(){
        return artMotor1.getEncoder().getPosition();
    }

    public double getFeedSpeed(){
        return feedMotor.getAppliedOutput();
    }

    public void setIntakeSpeed(double speed) {
        feedMotor.set(speed);
    }
    

    

    positions p;

    public double getPositionValue(){
        return p == positions.LOWER ? LOWER : UPPER;
    }

    public Command setPosition(positions po){
        return this.runOnce(() -> p = po);
    }

    public positions getPosition(){
        return p;
    }
    
    public enum positions{
        UPPER,
        LOWER
    }

    public void setBrakeState(int index){
        if(index == 1) artMotor1.setIdleMode(IdleMode.kCoast);
        else artMotor1.setIdleMode(IdleMode.kBrake);
    }

    public void ResetEncoder(){
        artMotor1.getEncoder().setPosition(UPPER);
    }

    public boolean getArtSensorError(){
        return artMotor1.getFault(FaultID.kSensorFault);
    }
    public boolean getArtMotorError(){
        return artMotor1.getFault(FaultID.kMotorFault);
    }

    public double getArtErrors(){
        return artMotor1.getFaults();
    }  
    
    public boolean getFeedSensorError(){
        return feedMotor.getFault(FaultID.kSensorFault);
    }
    public boolean getFeedMotorError(){
        return feedMotor.getFault(FaultID.kMotorFault);
    }

    public double getFeedErrors(){
        return feedMotor.getFaults();
    }    
}
