package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeController extends SubsystemBase {
    CANSparkMax feedMotor, artMotor1, artMotor2;

    public IntakeController(){
        feedMotor = new CANSparkMax(0, null);
        artMotor1 = new CANSparkMax(0, null);
        artMotor2 = new CANSparkMax(0, null);
    }
}
