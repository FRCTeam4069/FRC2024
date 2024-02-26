package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

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

    private final double kP = 4.5, kI = 0, kD = 0.01;
    private PIDController controller;

    public ShooterRotationController(){
        left = new CANSparkMax(5 , MotorType.kBrushless);
        right = new CANSparkMax(6, MotorType.kBrushless);

        encoder = new DutyCycleEncoder(0);
        left.setIdleMode(IdleMode.kBrake);
        right.setIdleMode(IdleMode.kBrake);

        left.setInverted(false);
        right.setInverted(false);

        

        //encoder.reset();

        controller = new PIDController(kP, kI, kD);

        

    }

    @Override
    public void periodic() {
        /* 
        System.out.println(getTargetAngle());
        System.out.println("right: " + right.get());
        System.out.println("left: " + left.get());
        System.out.println("encoder" + encoder.getAbsolutePosition());
        */

    }

    public void drive(double speed) {
        left.set(-speed);
        right.set(speed);
    }

    // public void runWithSpeed(double speed){
    //     right.set(speed);
    // }

    public void goToAngle(){
        right.set(controller.calculate(getAngle(), Math.toRadians(90-40)));
        left.set(-controller.calculate(getAngle(), Math.toRadians(90-40)));
        // right.set(0.1);
        // left.set(-0.1);
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

    public Command setSpeed(DoubleSupplier speed) {
        return this.run(() -> drive(speed.getAsDouble()));
    }

    public double getTargetAngle(){
        if(angle == shooterAngles.NINTEY) return (Math.PI/2);
        else if(angle == shooterAngles.NEG_NINTEY) return -Math.PI/2;
        return 0;
    }
    public enum shooterAngles{
        NINTEY,
        ZERO,
        NEG_NINTEY
    }
    
}
