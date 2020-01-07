package frc.robot.actions.spinny;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Spinny;

public class ManualSpin extends Action{

    private double speed;

    public ManualSpin(double speed){
        this.speed = speed;
    }

    @Override
    public void onStart() {
        System.out.println("spinny with power " + speed);
        Spinny.getInstance().updateManualSpin(speed);

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