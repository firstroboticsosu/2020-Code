package frc.robot.actions.ramp;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Ramp;

public class Door extends Action {

    boolean wantClosed = false;

    public Door(boolean wantClosed) {
        this.wantClosed = wantClosed;
    }

    @Override
    public void onStart() {
        Ramp.getInstance().setDoor(wantClosed);
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
