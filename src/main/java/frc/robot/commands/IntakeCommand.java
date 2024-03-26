package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.IntakeController;
import frc.robot.subsystems.IntakeController.positions;

public class IntakeCommand extends Command{
    IntakeController intake;

    private final double kP = 0.0155, kI = 0, kD = 0;
    private PIDController controller = new PIDController(kP, kI, kD);
    private double setpoint;
    private double speed;
    private boolean setSpeed = false;

    public IntakeCommand(IntakeController intake, positions position){
        this.intake = intake;
        if (position == positions.UPPER) {
            setpoint = IntakeConstants.UPPER_POSITION - 3;
        } else {
            setpoint = IntakeConstants.LOWER_POSITION - 6;
        }
        intake.setPosition(position);
        // intake.ResetEncoder();
    }

    public IntakeCommand(IntakeController intake, positions position, double speed){
        this.intake = intake;
        this.speed = speed;
        this.setSpeed = true;
        if (position == positions.UPPER) {
            setpoint = IntakeConstants.UPPER_POSITION - 3;
        } else {
            setpoint = IntakeConstants.LOWER_POSITION - 6;
        }
        intake.setPosition(position);
        // intake.ResetEncoder();
    }

    @Override
    public void initialize() {
        if (setSpeed) {
            if (speed == 0) {
                intake.stopFeed();
            } else {
                intake.setIntakeSpeed(speed);

            }
        }
    }

    @Override
    public void execute(){
        intake.driveArt(controller.calculate(intake.getEncoder(), setpoint));
    }

    @Override
    public void end(boolean interrupted){
        intake.stopArt();
    }

    @Override
    public boolean isFinished(){
        return MathUtil.isNear(setpoint, intake.getEncoder(), 5);
    }
    
}
