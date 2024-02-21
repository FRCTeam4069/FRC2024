package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs;

public class IntakeController extends SubsystemBase {
    CANSparkMax feedMotor, artMotor1;

    public IntakeController(){
        feedMotor = new CANSparkMax(DeviceIDs.INTAKE_FEED, MotorType.kBrushless);
        artMotor1 = new CANSparkMax(DeviceIDs.INTAKE_ARTICULATE, MotorType.kBrushless);

        feedMotor.setIdleMode(IdleMode.kBrake);
        artMotor1.setIdleMode(IdleMode.kBrake);

        artMotor1.setSoftLimit(SoftLimitDirection.kForward, 10000);
        artMotor1.setSoftLimit(SoftLimitDirection.kReverse, 100000);
        
    }

    public void driveFeed(){
        feedMotor.set(1);
    }
    public void stopFeed(){
        feedMotor.stopMotor();
    }
    public void driveArt(double speed){

    }
}
