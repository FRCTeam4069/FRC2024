package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DeviceIDs;
import frc.robot.constants.IntakeConstants;

public class ShooterRotationController extends SubsystemBase {
    private CANSparkMax left, right;
    private DutyCycleEncoder encoder;

    private final double kP = 0.01, kI = 0, kD = 0;
    private PIDController controller;

    public ShooterRotationController(){
        // //left = new CANSparkMax(DeviceIDs.SHOOTER_ARTICULATE_LEFT +45, MotorType.kBrushless);
        // right = new CANSparkMax(DeviceIDs.SHOOTER_ARTICULATE_RIGHT+45, MotorType.kBrushless);

        // encoder = new DutyCycleEncoder(0);
        // left.setIdleMode(IdleMode.kBrake);
        // right.setIdleMode(IdleMode.kBrake);

        // left.setInverted(true);

        // left.follow(right);

        // //encoder.reset();

        // controller = new PIDController(kP, kI, kD);

    }

    public void runWithSpeed(double speed){
        right.set(speed);
    }

    public void goToAngle(){
        right.set(controller.calculate(getAngle(), getTargetAngle()));
    }

    public double getEncoder(){
        return encoder.getAbsolutePosition();
    }

    public double getAngle(){
        return (encoder.getAbsolutePosition() - IntakeConstants.ShooterOffset) * (2*Math.PI);
    }

    shooterAngles angle;

    public Command setAngle(shooterAngles a){
        return this.runOnce(() -> angle = a);
    }

    public double getTargetAngle(){
        if(angle == shooterAngles.NINTEY) return 90;
        else if(angle == shooterAngles.NEG_NINTEY) return -90;
        return 0;
    }
    public enum shooterAngles{
        NINTEY,
        ZERO,
        NEG_NINTEY
    }
    
}
