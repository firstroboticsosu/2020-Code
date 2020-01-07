package frc.robot.actions.spinny;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Spinny;

public class AutoSpin extends Action{

    public AutoSpin(){
        Spinny.getInstance().initAutoSpin();
    }

    @Override
    public void onStart() {
        // TODO Auto-generated method stub

    }

    @Override
    public void onLoop() {
        // TODO Auto-generated method stub

    }

    @Override
    public boolean isFinished() {
        return Spinny.getInstance().autoSpinComplete();
    }

    @Override
    public void onStop() {
        Spinny.getInstance().deactivate();
    }

}