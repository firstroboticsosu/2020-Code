package frc.lib.statemachine;

import edu.wpi.first.wpilibj.Timer;

import java.util.LinkedList;

public class ActionGroup {
    private LinkedList<Action> group;
    private double t_Start, t_Timeout;

    ActionGroup(Action[] actions, long timeout_ms) {
        t_Timeout = (double) timeout_ms / 1000.0;
        group = new LinkedList<>();
        for (Action action : actions) {
            group.add(action);
        }

    }

    ActionGroup(Action action, long timeout_ms) {
        if(action  == null) throw new IllegalArgumentException("Attempted to pass a null action into a action group");
        t_Timeout = (double) timeout_ms / 1000.0;
        group = new LinkedList<>();
        group.add(action);
    }

    /**
     * records start time of state then executes all actions onStart
     */
    public void onStart() {
        t_Start = Timer.getFPGATimestamp();
        group.forEach(Action::onStart);
    }

    /**
     * runs the onloop code of all actions in the group
     * <p>executes in order added to group
     */
    public void onLoop() {
        group.forEach(Action::onLoop);
    }

    /**
     * determines when to begin state advancement
     * <p>handles self terminated actions automatically
     * @return true when all actions are finished
     */
    public boolean isFinished() {
        if (t_Start + t_Timeout <= Timer.getFPGATimestamp()) return true;
        boolean temp = true;
        for (Action action : group) {
            if(action.isFinished()){
                action.doStop();
            }
            else{
                temp = false;
            }
        }
        return temp;
    }

    /**
     * forceful exit for all actions in the group
     */
    public void onStop() {
        group.forEach(Action::doStop);
    }
}

