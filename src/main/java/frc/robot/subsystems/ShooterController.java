package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;

import com.ctre.phoenix6.controls.VelocityVoltage;

import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import static edu.wpi.first.units.MutableMeasure.mutable;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;


public class ShooterController extends SubsystemBase {
    TalonFX talon1;
    TalonFX talon2;
    SimpleMotorFeedforward feedforward;
    final VelocityVoltage v = new VelocityVoltage(0);
    Slot0Configs config = new Slot0Configs();

    private final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Angle> angle = mutable(Rotations.of(0));
    private final MutableMeasure<Velocity<Angle>> velo = mutable(RotationsPerSecond.of(0));

    SysIdRoutine routine;

    private DutyCycleEncoder encoder;
        

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
    }

    public void ShootWithPos(double x, double theta){
        double leftSpeed = x;
        double rightSpeed = x/2;
        
        v.Slot = 0;
        talon1.setControl(v.withVelocity(leftSpeed));
        talon2.setControl(v.withVelocity(rightSpeed));
    }


    public boolean atSpeed(){
        if(talon1.getVelocity().getValueAsDouble() >= 75 && talon1.getVelocity().getValueAsDouble() <= 85) return true;
        else return false;
    }

    public boolean isShooting(){
        return talon1.getVelocity().getValueAsDouble() > 10;
    }

    public double getFasterVelocity(){
        return talon1.getVelocity().getValueAsDouble();
    }
    public double getSlowerVelocity(){
        return talon2.getVelocity().getValueAsDouble();
    }

    public void driveWithCustomSpeed(double leftVel, double rightVel){
        v.Slot = 0;
        talon1.setControl(v.withVelocity(leftVel));
        talon2.setControl(v.withVelocity(-rightVel));
    }
    
   
}

