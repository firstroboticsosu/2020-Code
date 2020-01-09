package frc.lib.statemachine;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

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

    public static Command toCommand2(Action action){
        return new Command() {
            public void initialize(){
                action.onStart();
            }

            public void execute(){
                action.onLoop();
            }

            public boolean isFinished() {
                return action.isFinished();
            }

            public void end(boolean interrupted){
                action.onStop();
            }

            public Set<Subsystem> getRequirements(){
                return new HashSet<Subsystem>();
            }
        };
    }
}
