package frc.robot.actions.ramp;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Ramp;

public class Intake extends Action {

    double power;

    public Intake(double power) {
        this.power = power;
    }

    @Override
    public void onStart() {
        System.out.println("spinning roller");
        Ramp.getInstance().spin(power);
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
        Ramp.getInstance().stopSpinning();
    }
}
