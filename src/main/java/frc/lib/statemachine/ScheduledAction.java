package frc.lib.statemachine;

import edu.wpi.first.wpilibj.Timer;

public class ScheduledAction {

    private ActionState state;
    private Action action;
    private double t_Start, t_Timeout;

    public static final long NO_TIMEOUT = -1;

    /**
     * creates a new scheduled action
     * @param action the parent action with defined behavior
     * @param timeout_ms timeout in miliseconds for the scheduled action (can be disabled if set to -1)
     */
    public ScheduledAction(Action action, long timeout_ms){
        if(action  == null) throw new IllegalArgumentException("Attempted to pass a null action into a scheduled action");
        state = ActionState.READY;
        t_Timeout = (double) timeout_ms / 1000.0;
        this.action = action;
    }

    /**
     * creates a new scheduled action with no timeout
     * @param action the parent action with defined behavior
     */
    public ScheduledAction(Action action){
        if(action  == null) throw new IllegalArgumentException("Attempted to pass a null action into a scheduled action");
        state = ActionState.READY;
        t_Timeout = NO_TIMEOUT;
        this.action = action;
    }

    /**
     * Code that runs when the action is first called to start.
     * Runs the user defined onStart code and sets the state to STARTING.
     */
    public void onStart(){
        state = ActionState.STARTING;
        t_Start = Timer.getFPGATimestamp();
        action.onStart();
    }

    /**
     * Code that runs when the loop function of the action should be executing.
     * Runs the user defined onLoop code and sets the state to RUNNING.
     */
    public void onLoop(){
        state = ActionState.RUNNING;
        action.onLoop();
    }

    /**
     * 
     * @return
     */
    public boolean isFinished(){
        if(action.isFinished() || 
        t_Timeout > 0 &&
        t_Start + t_Timeout <= Timer.getFPGATimestamp()){
            state = ActionState.STOPPING;
            return true;
        }
        return false;
    }

    /**
     * Code that runs when the action has been called to stop.
     * Runs the user defined code and sets the current state to DEAD
     */
    public void onStop(){
        action.onStop();
        state = ActionState.DEAD;
    }

    /**
     * gets the current state of this scheduled action
     * @return the current state of the scheduled action
     */
    public ActionState getState(){
        return state;
    }

    /**
     * Sets the current state of the action to stopping.
     * This will adavance the state in the scheduler.
     */
    public void cancel(){
        state = ActionState.STOPPING;
    }

    /**
     * Resets the current state of the command for scheduling purposes 
     * to allow it to be reused. Will set the state back to READY
     */
    public void reset(){
        state = ActionState.READY;
    }

    /**
     * Will determine if the action is currently in a startable state
     * @return true if the current state is READY
     */
    public boolean startable(){
        return state == ActionState.READY;
    }

    /**
     * Will determine if the action is currently in a stopable state
     * @return true if the current state is not READY, STOPPING or DEAD
     */
    public boolean stopable(){
        return state != ActionState.READY || state != ActionState.STOPPING || state != ActionState.DEAD;
    }

    public enum ActionState{
        READY,
        STARTING,
        RUNNING,
        STOPPING,
        DEAD;

        public String toString(){
            return name().charAt(0) + name().substring(1).toLowerCase();
        }
    }

}