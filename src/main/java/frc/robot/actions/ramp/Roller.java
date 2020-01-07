package frc.robot.actions.ramp;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Ramp;

public class Roller extends Action {

    boolean wantDown;

    public Roller(boolean wantDown) {
        this.wantDown = wantDown;
    }

    @Override
    public void onStart() {
        Ramp.getInstance().setRoller(wantDown);
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
