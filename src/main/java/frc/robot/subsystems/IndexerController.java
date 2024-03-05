package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs;

public class IndexerController extends SubsystemBase {
    CANSparkMax m1;
    private final edu.wpi.first.wpilibj.AnalogInput pes;

    public IndexerController(){
        m1 = new CANSparkMax(DeviceIDs.FEEDER, MotorType.kBrushless);
        pes = new edu.wpi.first.wpilibj.AnalogInput(0);
        m1.setIdleMode(IdleMode.kBrake);
    }

    public void feedShooter(){
        m1.set(0.45);
    }

    public void unFeedShooter(){
        m1.set(-0.45);
    }
    public void stop(){
        m1.stopMotor();
    }

    public boolean getPhotoReading(){
        if(pes.getVoltage() > 0.7) return true;
        return false;
    }

    public double getCurrent(){
        return m1.getOutputCurrent();
    }

    public void setCustomSpeed(double speed){
        m1.set(speed);
    }

    public void slowFeed(){
        m1.set(0.3);
    }
    
}
