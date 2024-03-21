package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerController;
import frc.robot.subsystems.IntakeController.positions;

public class BetterIndexerCommand extends Command {
    private IndexerController indexer;
    private State state;
    private double setpoint;
    private PIDController pid;
    private BooleanSupplier shoot;
    public BetterIndexerCommand(IndexerController indexer, BooleanSupplier shoot) {
        this.indexer = indexer;
        state = State.INITIAL;
        pid = new PIDController(0.055, 0.0, 0.0);
        this.shoot = shoot;
    }

    enum State {
        INITIAL,
        PID,
        IN
    }

    @Override
    public void initialize() {
        indexer.setPosition(0);
    }

    @Override
    public void execute() {
        var isLoaded = indexer.getPhotoReading();
        switch (state) {
            case INITIAL:
                if (isLoaded) {
                    state = State.PID;
                    setpoint = indexer.getPosition();
                    indexer.stop();
                    break;
                }
                indexer.feedShooter();
                break;
            case IN:
                if (shoot.getAsBoolean() || !MathUtil.isNear(setpoint, indexer.getPosition(), 5.0)) {
                    state = State.INITIAL;
                    break;
                }
                indexer.stop();
                break;
            case PID:
                if (isLoaded && Math.abs(indexer.getVelocity()) < 1.0 || MathUtil.isNear(setpoint, indexer.getPosition(), 1.0)) {
                    state = State.IN;
                    setpoint = indexer.getPosition();
                    indexer.stop();
                    break;
                }
                if (indexer.getPosition() - setpoint < -8 && indexer.getPosition() - setpoint > 50 || shoot.getAsBoolean()) {
                    state = State.INITIAL;
                    break;
                }
                indexer.setCustomSpeed(pid.calculate(indexer.getPosition(), setpoint));
                break;
            default:
                break;
        }

        SmartDashboard.putString("indexer state", state.name());
    }
    @Override
    public boolean isFinished() {
        //return state == State.IN && Math.abs(indexer.getVelocity()) < 1.0;
        return false;
    }
    @Override
    public void end(boolean interrupted) {
        indexer.stop();
    }

    
}
    