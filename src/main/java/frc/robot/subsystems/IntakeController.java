package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs;
import frc.robot.constants.IntakeConstants;

public class IntakeController extends SubsystemBase {
    CANSparkMax feedMotor, artMotor1;

    private final double LOWER = IntakeConstants.LOWER_POSITION - 3, 
                         UPPER = IntakeConstants.UPPER_POSITION - 3;


    public IntakeController(){
        feedMotor = new CANSparkMax(DeviceIDs.INTAKE_FEED, MotorType.kBrushless);
        artMotor1 = new CANSparkMax(DeviceIDs.INTAKE_ARTICULATE, MotorType.kBrushless);

        feedMotor.setIdleMode(IdleMode.kBrake);
        artMotor1.setIdleMode(IdleMode.kBrake);

        artMotor1.setSoftLimit(SoftLimitDirection.kForward, IntakeConstants.UPPER_POSITION);
        artMotor1.setSoftLimit(SoftLimitDirection.kReverse, IntakeConstants.LOWER_POSITION);
        
    }

    public void driveFeed(){
        feedMotor.set(1);
    }
    public void stopFeed(){
        feedMotor.stopMotor();
    }
    public void driveArt(double speed){
        artMotor1.set(speed);
    }
    public void stopArt(){
        artMotor1.stopMotor();
    }

    public double getEncoder(){
        return artMotor1.getEncoder().getPosition();
    }
}
