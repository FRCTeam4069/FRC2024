package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerController extends SubsystemBase {
    CANSparkMax m1, m2;
    private AnalogInput pes;

    public IndexerController(){
        m1 = new CANSparkMax(Constants.INDEXER_1_ID, MotorType.kBrushless);
        m2 = new CANSparkMax(Constants.INDEXER_2_ID, MotorType.kBrushless);

        pes = new AnalogInput(Constants.PHOTO_ELECTRIC_SENSOR_PORT);

        m1.setIdleMode(IdleMode.kBrake);
        m2.setIdleMode(IdleMode.kBrake);

        m2.setInverted(true);

        m2.follow(m1);
    }

    public void feedShooter(){
        m1.set(0.7);
    }
    public void stop(){
        m1.stopMotor();
    }

    public double getPhotoReading(){
        return pes.getValue();
    }
    
}
