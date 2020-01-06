package frc.robot.actions.ramp;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Ramp;

public class Polycord extends Action {

    boolean isRunning = false;
    boolean runUp = true;

    public Polycord(boolean isRunning, boolean runUp) {
        this.isRunning = isRunning;
        this.runUp = runUp;
    }

    @Override
    public void onStart() {
        if (isRunning) {
            if (runUp) {
                Ramp.getInstance().spinUp();
            }
            else {
                Ramp.getInstance().spinDown();
            }
        }
        else {
            Ramp.getInstance().stopSpinning();
        }
    }

    @Override
    public void onLoop() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void onStop() {

    }
}
