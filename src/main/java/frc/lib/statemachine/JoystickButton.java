package frc.lib.statemachine;

import edu.wpi.first.wpilibj.GenericHID;

public class JoystickButton extends Trigger {

    protected GenericHID joystick;
    protected int buttonNum;

    /**
     * Creates a new instance of a joystick button as a trigger for the scheduler.
     * This instance is not completely configured yet. It must be given an action and
     * trigger type before use.
     * @param joystick the joystick to reference
     * @param buttonNumber indexed number for the selected button
     */
    public JoystickButton(GenericHID joystick, int buttonNumber){
        this.joystick = joystick;
        buttonNum = buttonNumber;
    }

    /**
     * Creates a new instance of a joystick button as a trigger for the scheduler.
     * This one creates a ready for use trigger that can be immediately registered.
     * @param joystick the joystick to reference
     * @param buttonNumber indexed number for the selected button
     * @param action action that the trigger will perform when the condition is met
     * @param triggerType the type of triggering to use
     */
    public JoystickButton(GenericHID joystick, int buttonNumber, Action action, TriggerType triggerType){
        this(joystick, buttonNumber);
        triggerAction = new ScheduledAction(action);
        type = triggerType;    
    }

    /**
     * gets the curent trigger status
     * @return the current trigger status of the button
     */
    public boolean get(){
        return joystick.getRawButton(buttonNum);
    }

    /**
     * Starts the given command whenever the trigger just becomes active.
     * @param action action that the trigger will perform when the condition is met
     * @param triggerType the type of triggering to use
     */
    public void whenPressed(final Action action) {
        triggerAction = new ScheduledAction(action);
        type = TriggerType.WHENACTIVE;     
    }
  
    /**
     * runs the action while the button is held.
     * @param action action that the trigger will perform when the condition is met
     * @param triggerType the type of triggering to use
     */
    public void whileHeld(final Action action) {
        triggerAction = new ScheduledAction(action);
        type = TriggerType.WHILEACTIVE;
    }
  
    /**
     * Starts the action when the button becomes inactive.
     * @param action action that the trigger will perform when the condition is met
     * @param triggerType the type of triggering to use
     */
    public void whenReleased(final Action action) {
        triggerAction = new ScheduledAction(action); 
        type = TriggerType.WHENINACTIVE;
    }
  
    /**
     * Starts the action when the button becomes inactive.
     * @param action action that the trigger will perform when the condition is met
     * @param triggerType the type of triggering to use
     */
    public void whileReleased(final Action action) {
        triggerAction = new ScheduledAction(action);  
        type = TriggerType.WHILEINACTIVE;
    }

}