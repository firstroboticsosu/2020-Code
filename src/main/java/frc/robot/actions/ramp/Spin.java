package frc.robot.actions.ramp;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Ramp;

public class Spin extends Action {

    boolean forwards;

    public Spin(boolean forwards) {
        this.forwards = forwards;
    }

    @Override
    public void onStart() {
        Ramp.getInstance().spin(forwards);
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
        Ramp.getInstance().stopSpinning();
    }
}
