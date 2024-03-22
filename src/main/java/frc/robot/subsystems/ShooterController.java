package frc.robot.subsystems;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.ctre.phoenix6.configs.Slot0Configs;

import com.ctre.phoenix6.controls.VelocityVoltage;

import com.ctre.phoenix6.hardware.TalonFX;

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

    
    SysIdRoutine routine;

    //private DutyCycleEncoder encoder;
        

    public ShooterController(){
        talon1 = new TalonFX(9);
        talon2 = new TalonFX(8);


        var slot0Configs = new Slot0Configs();
        slot0Configs.kV = 0.12273;
        slot0Configs.kP = 0.11;
        slot0Configs.kI = 0.48;
        slot0Configs.kD = 0.01;

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
    public void stop(){
        talon1.set(0);
        talon2.set(0);
        targetSpeed = 0;
    }

    public void ShootWithPos(double x, double theta){
        double leftSpeed = x;
        double rightSpeed = x;
        
        v.Slot = 0;
        talon1.setControl(v.withVelocity(leftSpeed));
        talon2.setControl(v.withVelocity(rightSpeed));
    }


    public boolean atSpeed(){
        if(talon1.getVelocity().getValueAsDouble() < 5) return false; 
        else if(MathUtil.isNear(talon1.getVelocity().getValueAsDouble(), targetSpeed, 2)) return true; 
<<<<<<< HEAD
=======
        return false;
    }

    public boolean atSpeed(double tolerance){
        if(talon1.getVelocity().getValueAsDouble() < 5) return false; 
        else if(MathUtil.isNear(talon1.getVelocity().getValueAsDouble(), targetSpeed, tolerance)) return true; 
>>>>>>> origin/main
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
    double targetSpeed = 0;

    public void driveWithCustomSpeed(double leftVel, double rightVel){
        v.Slot = 0;
        targetSpeed = leftVel;

        if(leftVel <= 30){
            talon1.setControl(v.withVelocity(leftVel));
            talon2.setControl(v.withVelocity(-leftVel));
        }
        else{
            talon1.setControl(v.withVelocity(leftVel));
            talon2.setControl(v.withVelocity(-rightVel/1.5));
        }

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Right Shooter Speed", talon2.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Left  Shooter Speed", talon1.getVelocity().getValueAsDouble());

    }
    
   
}

