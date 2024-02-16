package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;

import com.ctre.phoenix6.controls.VelocityVoltage;

import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterController extends SubsystemBase {
    TalonFX talon1;
    TalonFX talon2;
    SimpleMotorFeedforward feedforward;
    final VelocityVoltage v = new VelocityVoltage(0);
    Slot0Configs config = new Slot0Configs();

    public ShooterController(){
        talon1 = new TalonFX(Constants.SHOOTER_1_ID);
        talon2 = new TalonFX(Constants.SHOOTER_2_ID);

        talon1.setInverted(true);

        var slot0Configs = new Slot0Configs();
        slot0Configs.kV = 0.11811;
        slot0Configs.kP = 0.11;
        slot0Configs.kI = 0.48;
        slot0Configs.kD = 0.01;

        talon1.getConfigurator().apply(slot0Configs, 0.050);
        talon2.getConfigurator().apply(slot0Configs, 0.050);


        //feedforward = new SimpleMotorFeedforward(0.79972, );
    }

    public void drive(){
        v.Slot = 0;
        talon1.setControl(v.withVelocity(80));
        talon2.setControl(v.withVelocity(40));
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
   
}

