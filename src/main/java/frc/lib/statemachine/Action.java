package frc.lib.statemachine;

import edu.wpi.first.wpilibj.command.Command;

public abstract class Action {

    private boolean hasStopped = false;

    /**
     * code to run on action start
     */
    public abstract void onStart();

    /**
     * code to run while action loops
     * <p>approx every 20 miliseconds
     */
    public abstract void onLoop();

    /**
     * method that tells the state machine the action is finished earlier than the scheduler
     * @return true when action is ready to self terminate
     */
    public abstract boolean isFinished();

    /**
     * code to run when the action has ben called by the state machine to stop
     */
    public abstract void onStop();

    /**
     * non implemented method by child class
     * <p>prevents onstop from being called multiple times
     */
    public void doStop(){
        if(!hasStopped){
            onStop();
            hasStopped = true;
        }
    }

    /**
     * converts an action to a wpilib command for buttons
     */
    @Deprecated
    public static Command toCommand(Action action){
        return new Command() {

            protected void initialize(){
                action.onStart();
            }

            protected void execute(){
                action.onLoop();
            }

            protected boolean isFinished() {
                return action.isFinished();
            }

            protected void end(){
                action.onStop();
            }
        };
    }
}
