package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.imu.SwerveIMU;

public class Gyro extends SubsystemBase {
    SwerveIMU _pigeon;

    public Gyro(SwerveIMU swerve_pigeon) {
        // this will need to change
        _pigeon = swerve_pigeon;
    }

    public void reset () {
        // _pigeon.getRotation3d().
    }

    public double angleRadians() {
        // this might have to be pi instead (depending on if it's negative or not)
        return _pigeon.getRawRotation3d().getZ() % (2 * Math.PI);
    }

    public double angleDegrees() {
        return angleRadians() / Math.PI * 180;
    }

    public Translation3d getAcceleration() {
        return _pigeon.getAccel().get();
    }
}
