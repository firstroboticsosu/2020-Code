package frc.robot.actions;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachine;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.PoseEstimator;

public class AStopAction extends Action {
    boolean finished = false;

    @Override
    public void onStart() {
        StateMachine.assertStop();
        Drive.getInstance().onStop();
        PoseEstimator.getInstance().onStop();
        finished = true;
    }

    @Override
    public void onLoop() {

    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void onStop() {

    }
}
