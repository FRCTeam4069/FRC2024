package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase {
    Pigeon2 _pigeon;

    public Gyro() {
        // this will need to change
        _pigeon = new Pigeon2(0, null);
    }

    public void reset () {
        _pigeon.setYaw(0);
    }

    public double angleDegrees() {
        // this might have to be % 180 instead (depending on if it's negative or not)
        return _pigeon.getYaw().getValue() % 360;
    }

    public double angleRadians() {
        return angleDegrees() / 180 * Math.PI;
    }
    public double accelerationDegrees() {
        return _pigeon.getAccelerationX().getValue();
    }

    public double accelerationRadians() {
        return accelerationDegrees() / 180 * Math.PI;
    }
}
