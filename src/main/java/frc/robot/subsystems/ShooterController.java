package frc.robot.subsystems;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;

import com.ctre.phoenix6.hardware.TalonFX;
import com.reduxrobotics.sensors.canandcolor.digout.SlotComparison;
import com.revrobotics.CANSparkBase.FaultID;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;


public class ShooterController extends SubsystemBase {
    TalonFX talon1;
    TalonFX talon2;
    SimpleMotorFeedforward feedforward;
    final VelocityVoltage v = new VelocityVoltage(0);
    Slot0Configs config = new Slot0Configs();
    //Slot1Configs slowConfig = new Slot1Configs();

    double targetSpeed = 0;
    
    SysIdRoutine routine;

    Slot0Configs slot0Configs = new Slot0Configs();
    Slot1Configs slot1Configs = new Slot1Configs();
    //private DutyCycleEncoder encoder;
        

    public ShooterController(){
        talon1 = new TalonFX(9);
        talon2 = new TalonFX(8);


        
        slot0Configs.kV = 0.12273;
        slot0Configs.kP = 0.155;
        slot0Configs.kI = 0.13;
        slot0Configs.kD = 0.0272;

        
        slot1Configs.kV = 0.12273;
        slot1Configs.kP = 0.11;
        slot1Configs.kI = 0.48;
        slot1Configs.kD = 0.01;

        // var slot1Configs = new Slot0Configs();
        // slot0Configs.kV = 0.12273;
        // slot0Configs.kP = 0.11;
        // slot0Configs.kI = 0.48;
        // slot0Configs.kD = 0.01;
        

        talon1.getConfigurator().apply(slot0Configs, 0.050);
        talon2.getConfigurator().apply(slot0Configs, 0.050);


        //feedforward = new SimpleMotorFeedforward(0.79972, );

       talon1.setInverted(true);

       

    }

    public void drive(){
        v.Slot = 0;
        talon1.setControl(v.withVelocity(60));
        talon2.setControl(v.withVelocity(-30));
    }

    public void switchSlotConfig(SlotConfigs c){
        if (c == SlotConfigs.SLOW){
            v.Slot = 1;
            configRunning = 1;
        }
        else if (c == SlotConfigs.FAST){
            v.Slot = 0;
            configRunning = 0;
        }
       
    }

    public enum SlotConfigs{
        SLOW,
        FAST
    }

    public void stop(){
        talon1.set(0);
        talon2.set(0);
    }

    public void ShootWithPos(double x, double theta){
        double leftSpeed = x;
        double rightSpeed = x/2;
        
        v.Slot = 0;
        talon1.setControl(v.withVelocity(leftSpeed));
        talon2.setControl(v.withVelocity(rightSpeed));
    }


    public boolean atSpeed(){
        if(talon1.getVelocity().getValueAsDouble() < 5) return false; 
        else if(MathUtil.isNear(talon1.getVelocity().getValueAsDouble(), targetSpeed, 0.8)) return true; 
        return false;
    }

    public boolean atSpeed(double tolerance){
        if(talon1.getVelocity().getValueAsDouble() < 5) return false; 
        else if(MathUtil.isNear(talon1.getVelocity().getValueAsDouble(), targetSpeed, tolerance)) return true; 
        return false;
    }

    public boolean isShooting(){
        return talon1.getVelocity().getValueAsDouble() > 3;
    }

    public double getFasterVelocity(){
        return talon2.getVelocity().getValueAsDouble();
    }
    public double getSlowerVelocity(){
        return talon2.getVelocity().getValueAsDouble();
    }

    public void driveWithCustomSpeed(double leftVel, double rightVel){
        switchSlotConfig(SlotConfigs.FAST);
        v.Slot = 0;

        targetSpeed = leftVel;

        if(leftVel <= 30){
            talon1.setControl(v.withVelocity(leftVel));
            talon2.setControl(v.withVelocity(-leftVel));
        }
        else{
            talon1.setControl(v.withVelocity(leftVel));
            talon2.setControl(v.withVelocity(-rightVel * .9));
        }

        SmartDashboard.putNumber("Right Shooter Speed", talon2.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Left  Shooter Speed", talon1.getVelocity().getValueAsDouble());
    }

    double configRunning = 0;

    public void driveWithSlowSpeed(double leftVel, double rightVel){

        //switchSlotConfig(SlotConfigs.SLOW);
        v.Slot = 1;
        targetSpeed = leftVel;

        if(leftVel <= 30){
            talon1.setControl(v.withVelocity(leftVel));
            talon2.setControl(v.withVelocity(-leftVel));
        }
        else{
            talon1.setControl(v.withVelocity(leftVel));
            talon2.setControl(v.withVelocity(-leftVel));
        }
    }
    
    public double getConfig(){
        
        return talon1.getClosedLoopSlot().getValueAsDouble();
        
    }
}
