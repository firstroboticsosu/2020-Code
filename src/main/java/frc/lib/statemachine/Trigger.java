package frc.lib.statemachine;

public abstract class Trigger {
  
    protected boolean wasActive = false;
    protected ScheduledAction triggerAction;
    protected TriggerType type;

    /**
     * Returns whether or not the trigger is active.
     *  This method will be called repeatedly when the trigger is scheduled
     *
     * @return whether or not the trigger condition is active.
     */
    public abstract boolean get();

    /**
     * determines if a trigger condition has been met
     * @return if the underlying trigger condition is met to
     * start the action
     */
    public boolean shouldStart(){
        boolean currentTriggerState = get();
        boolean shouldStart = false;
        switch (type) {
            case WHENACTIVE:
                shouldStart = !wasActive && currentTriggerState;
                break;
            case WHILEACTIVE:
                shouldStart = currentTriggerState;
                break;
            case WHENINACTIVE:
                shouldStart = wasActive && !currentTriggerState;
                break;
            case WHILEINACTIVE:
                shouldStart = !currentTriggerState;
                break;
            default:
                shouldStart = false;
        }
        wasActive = currentTriggerState;
        return shouldStart;
    }

    /**
     * determines if the underlying stop condition has been met
     * @return if the underlying cancel condition has been met in order
     * cancel the action and reset it
     */
    public boolean shouldStop(){
        boolean currentTriggerState = get();
        boolean shouldStop = true;
        switch (type) {
            case WHENACTIVE:
                shouldStop = wasActive && !currentTriggerState;
                break;
            case WHILEACTIVE:
                shouldStop = !currentTriggerState;
                break;
            case WHENINACTIVE:
                shouldStop = !wasActive && currentTriggerState;
                break;
            case WHILEINACTIVE:
                shouldStop = currentTriggerState;
                break;
            default:
                shouldStop = true;
        }
        wasActive = currentTriggerState;
        return shouldStop;
    }
    
    /**
     * gets the underlying scheduled action object
     * @return scheduled action registered by the trigger
     */
    public ScheduledAction getScheduledAction(){
        return triggerAction;
    }

    /**
     * returns the trigger type of this trigger
     * @returns the currently defined trigger type. Default is NONE
     */
    public TriggerType getTriggerType(){
        return type;
    }

    /**
     * resets triggers to their default state
     * will clear edge detection logic and reset the
     * underlying action to ready state
     */
    public void reset(){
        wasActive = false;
        triggerAction.onStop();
        triggerAction.reset();
    }

    //TODO support a toggle based trigger that starts on rising and cancels on falling
    /**
     * Trigger type enum to support active low,
     * active high, rising edge and falling edge trigger situations
     */
    public enum TriggerType{
        /** rising edge triggering */
        WHENACTIVE,

        /** active high triggering */
        WHILEACTIVE,

        /** falling edge triggering */
        WHENINACTIVE,

        /** active low triggering */
        WHILEINACTIVE,

        /** default triggering (no trigger status) */
        NONE;
    }
  
}  