package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.DeviceIDs;

public class AmpArm extends SubsystemBase {
    private Servo left, right;
    public static final double RETRACT = 0.20;
    public static final double EXTEND = 0.95;
    public AmpArm() {
        left = new Servo(DeviceIDs.AMPARM_LEFT);
        right = new Servo(DeviceIDs.AMPARM_RIGHT);

        setAngle(RETRACT);

    }

    /**
     * 
     * @param angle 0.0 to 1.0
     */
    public void setAngle(double angle) {
        left.set(MathUtil.clamp(angle, RETRACT, EXTEND));
        right.set(MathUtil.clamp(angle, RETRACT, EXTEND));
    }

    /**
     * Sets the angle of the arm (in 0.0 to 1.0) and waits so other things don't break it
     * @param angle
     * @return 
     */
    public Command setAngleCommand(double angle) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> setAngle(angle)),
                new WaitCommand(0.2)
            );
    }

    @Override
    public void periodic() {

    }
    
}
