package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ClimberSubsystem extends SubsystemBase {
    CANSparkMax climber;

    private static double pK = 0.1;
    private static double pG = 0.1;

    public ClimberSubsystem() {
        climber = new CANSparkMax(Constants.CLIMBER, MotorType.kBrushless);
    }

    public void raiseClimber() {
        climber.set(0.5);
    }

    public int positionDifference() {
        // need to do something about this
    }

    // a.k.a raise robot
    public void pullUp() {
        // may need to change this later
        climber.set(-pG + -pK * positionDifference);
    }
}
