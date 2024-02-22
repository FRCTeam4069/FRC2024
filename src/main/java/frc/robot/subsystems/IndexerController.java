package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.constants.DeviceIDs;

public class IndexerController extends SubsystemBase {
    CANSparkMax m1;
    private AnalogInput pes;

    public IndexerController(){
        m1 = new CANSparkMax(DeviceIDs.FEEDER, MotorType.kBrushless);
        //pes = new AnalogInput(Constants.PHOTO_ELECTRIC_SENSOR_PORT);
        m1.setIdleMode(IdleMode.kBrake);
    }

    public void feedShooter(){
        m1.set(0.7);
    }
    public void stop(){
        m1.stopMotor();
    }

    // public double getPhotoReading(){
    //     return pes.getValue();
    // }
    
}
