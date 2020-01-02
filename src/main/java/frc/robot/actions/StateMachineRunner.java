package frc.robot.actions;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachine;
import frc.lib.statemachine.StateMachineDescriptor;

public class StateMachineRunner extends Action {

    private StateMachineDescriptor state;

    public StateMachineRunner(StateMachineDescriptor state) {
        this.state = state;
    }

    public void onStart() {
        StateMachine.runMachine(state);
    }

    public void onLoop() {

    }

    public boolean isFinished() {
        return !StateMachine.isRunning();
    }

    public void onStop() {
        StateMachine.assertStop();
    }
}
