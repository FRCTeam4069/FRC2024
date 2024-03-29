package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerController;
import frc.robot.subsystems.IntakeController.positions;

public class BetterIndexerCommandWithStop extends Command {
    private IndexerController indexer;
    private State state;
    private double setpoint;
    private PIDController pid;
    public BetterIndexerCommandWithStop(IndexerController indexer) {
        this.indexer = indexer;
        state = State.INITIAL;
        pid = new PIDController(0.055, 0.0, 0.0);
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
                indexer.setCustomSpeed(0.65);
                break;
            case IN:
                if (!MathUtil.isNear(setpoint, indexer.getPosition(), 5.0)) {
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
                if (indexer.getPosition() - setpoint < -15 && indexer.getPosition() - setpoint > 30) {
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
        return state == State.IN && Math.abs(indexer.getVelocity()) < 0.5;
    }
    @Override
    public void end(boolean interrupted) {
        indexer.stop();
    }

    
}
    