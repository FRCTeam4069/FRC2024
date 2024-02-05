package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;

import com.ctre.phoenix6.controls.VelocityVoltage;

import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class Shooter {
    TalonFX talon1;
    TalonFX talon2;
    SimpleMotorFeedforward feedforward;
    final VelocityVoltage v = new VelocityVoltage(0);
    Slot0Configs config = new Slot0Configs();

    public Shooter(){
        talon1 = new TalonFX(9);
        talon2 = new TalonFX(8);

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
}
