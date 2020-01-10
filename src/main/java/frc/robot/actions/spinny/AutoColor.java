package frc.robot.actions.spinny;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Spinny;

public class AutoColor extends Action {

    @Override
    public void onStart() {
        Spinny.getInstance().initAutoColor();
    }

    @Override
    public void onLoop() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void onStop() {
        Spinny.getInstance().deactivate();
    }
}
