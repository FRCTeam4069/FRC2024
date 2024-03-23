package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.DeviceIDs;

public class IndexerController extends SubsystemBase {
    CANSparkMax m1;
    private final edu.wpi.first.wpilibj.AnalogInput pes;

    boolean past = false;

    public IndexerController(){
        m1 = new CANSparkMax(DeviceIDs.FEEDER, MotorType.kBrushless);
        pes = new edu.wpi.first.wpilibj.AnalogInput(0);
        m1.setIdleMode(IdleMode.kBrake);
        m1.setSmartCurrentLimit(40);
        m1.burnFlash();
    }

    public void periodic(){
        if(getPhotoReading()){
            past = true;
        }
        if(RobotContainer.shooter.isShooting()){
            past = false;
        }

        SmartDashboard.putNumber("indexer position", getPosition());
        SmartDashboard.putBoolean("colour sensor", getPhotoReading());
        SmartDashboard.putNumber("photoelectric sensor volts", pes.getVoltage());
    }

    public void feedShooter(){
        m1.set(0.65);
    }

    public void unFeedShooter(){
        m1.set(-0.45);
    }
    public void stop(){
        m1.stopMotor();
    }

    public boolean getPhotoReading(){
        return (pes.getVoltage() < 1.5);
    }

    public double getCurrent(){
        return m1.getOutputCurrent();
    }

    public void setCustomSpeed(double speed){
        m1.set(speed);
    }

    public void slowFeed(){
        m1.set(-0.2);
    }

    public boolean pastSensor(){
        return past;
    }

    public double getPosition() {
        return m1.getEncoder().getPosition();
    }

    public void setPosition(double position) {
        m1.getEncoder().setPosition(position);
    }

    public double getVelocity() {
        return m1.getEncoder().getVelocity();
    }
    
}
