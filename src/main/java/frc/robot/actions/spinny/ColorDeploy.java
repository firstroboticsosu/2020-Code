package frc.robot.actions.spinny;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Spinny;

public class ColorDeploy extends Action{

    private boolean wantDeploy = false;

    public ColorDeploy(boolean wantDeploy){
        this.wantDeploy = wantDeploy;
    }

    @Override
    public void onStart() {
        Spinny.getInstance().setColorDeploy(wantDeploy);
    }

    @Override
    public void onLoop() {    }

    @Override
    public boolean isFinished() {
        return Spinny.getInstance().getColorDeployed();
    }

    @Override
    public void onStop() {
    }

}