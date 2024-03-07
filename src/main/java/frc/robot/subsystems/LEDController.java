package frc.robot.subsystems;


import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDController extends SubsystemBase {
    private PWM blinkinLED;

    public LEDController() {
        blinkinLED = new PWM(0); // Replace '0' with the actual PWM port for your REV Blinkin

    }

    public Command setPattern(RevBlinkinPatterns pattern) {

                return this.runOnce(() -> blinkinLED.setPulseTimeMicroseconds((int)(pattern.getValue())));
    }

}


