package frc.robot.actions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Climb;

public class ClimbAction extends Action {

    @Override
    public void onStart() {
        Climb.getInstance().setWinch(0.3);
        Climb.getInstance().setRoller(0.5);
    }

    @Override
    public void onLoop() {
        // TODO Auto-generated method stub

    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void onStop() {
        Climb.getInstance().setWinch(0.0);
        Climb.getInstance().setRoller(0.0);
    }

}