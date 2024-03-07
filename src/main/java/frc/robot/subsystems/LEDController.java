package frc.robot.subsystems;


import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDController extends SubsystemBase {
    private final Spark blinkinLED;

    public LEDController() {
        blinkinLED = new Spark(0); // Replace '0' with the actual PWM port for your REV Blinkin

    }

    public Command setPattern(RevBlinkinPatterns pattern) {
        return this.runOnce(() -> blinkinLED.set((double) (pattern.getValue() * 255)));
    }

}


