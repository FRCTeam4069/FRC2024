package frc.robot.commands.drivebase;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Toggle {
    private BooleanSupplier trueCondition, falseCondition;
    private boolean state = true;

    public Toggle(BooleanSupplier trueCondition, BooleanSupplier falseCondition) {
        this.trueCondition = trueCondition;
        this.falseCondition = falseCondition;
    }

    public boolean getState() {
        if (falseCondition.getAsBoolean()) {
            state = false;
        }
        if (trueCondition.getAsBoolean()) {
            state = true;
        }

        SmartDashboard.putBoolean("state", state);

        return state;
    }
    
}
