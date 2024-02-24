package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs;

public class ShooterTest extends SubsystemBase {
    CANSparkMax left, right;

    public ShooterTest(){
        // left = new CANSparkMax(5, MotorType.kBrushless);
        // right = new CANSparkMax(6, MotorType.kBrushless);

        // left.setInverted(true);

        //left.follow(right);
    }

    public void drive(double speed){
        right.set(speed);
        left.set(speed);
    }
}
