package frc.lib.statemachine;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import edu.wpi.first.wpilibj.Timer;

public class Scheduler {

    private static final AtomicBoolean wantStop = new AtomicBoolean(true), isRunning = new AtomicBoolean(false);
    private static final List<ScheduledAction> scheduled = Collections.synchronizedList(new ArrayList<>());
    private static final List<Trigger> triggers = Collections.synchronizedList(new ArrayList<>());

    private volatile static double t_start;
    private static final double delay = 0.020;
    
    private static final Runnable Scheduler = () -> {
        try {
            //set the isRunning flag to true
            isRunning.set(true);

            while (!wantStop.get()){
                t_start = Timer.getFPGATimestamp();
                
                //process triggers
                for(Iterator<Trigger> iterator = triggers.iterator(); iterator.hasNext(); ){
                    Trigger trigger = iterator.next();

                    //make sure that the trigger has been initialized
                    if(trigger.getTriggerType() == Trigger.TriggerType.NONE) continue;

                    //iterate the list of triggers
                    switch(trigger.getScheduledAction().getState()){
                        case READY:
                            if(trigger.shouldStart()) trigger.getScheduledAction().onStart();
                            break;
                        case RUNNING:
                            //run the current onLoop code as part of the design pattern
                            trigger.getScheduledAction().onLoop();

                            //if is finished returns true, the state will change to stopping
                            trigger.getScheduledAction().isFinished();

                            //trigger condition override
                            if(trigger.shouldStop()){
                                //will force the action into a stopping state
                                trigger.getScheduledAction().cancel();
                            }
                            break;
                        case STOPPING:
                            //when the action is ready to stop run the onStop code
                            trigger.getScheduledAction().onStop();
                            //reset the state of the action back to ready
                            trigger.getScheduledAction().reset();
                            break;
                        default:
                            //ignore starting and dead cases
                    }
                }

                //iterate the list of actions and check status
                for(Iterator<ScheduledAction> iterator = scheduled.iterator(); iterator.hasNext(); ){
                    ScheduledAction action = iterator.next();
                    //manage execution of the current action
                    switch (action.getState()) {
                        case READY:
                            action.onStart();
                            break;
                        case RUNNING:
                            //run the onLoop code
                            action.onLoop();
                            //if is finished returns true, the state will change to stopping
                            action.isFinished();
                            break;
                        case STOPPING:
                            //run the on stop code
                            action.onStop();
                            break;
                        case DEAD:
                            //remove the action from the list
                            scheduled.remove(action);
                        default:
                            //ignore starting case
                    }
                }

                Timer.delay(delay - (Timer.getFPGATimestamp() - t_start));
            }
        }catch (Exception e){
            //error condition, a exception was thrown during scheduler execution
            System.out.println(e.getMessage());

        } finally {
            //reset triggers & clear the scheduled list

            for(Iterator<Trigger> iterator = triggers.iterator(); iterator.hasNext(); ){
                Trigger trigger = iterator.next();

                //reset all triggers to default state
                trigger.reset();
            }

            for(Iterator<ScheduledAction> iterator = scheduled.iterator(); iterator.hasNext(); ){
                ScheduledAction action = iterator.next();

                //kill all scheduled actions immediately
                action.onStop();
            }

            //clear the list of scheduled actions
            scheduled.clear();

            //set isRunning flag back to false
            isRunning.set(false);
        }
    };

    /**
     * Creates a single active scheduler thread to manage currently scheduled actions.
     */
    public static void runScheduler() {
        //entered a resettable state
        wantStop.set(false);

        //start the new scheduler thread
        Thread thread = new Thread(Scheduler);
        System.out.println("Scheduler starting");
        thread.start();
    }

    /**
     * Asserts the shutdown state of the scheduler. The scheduler will stop 
     * within the next iteration (the value of delay)
     */
    public static void assertStop(){
        if(!wantStop.get()){
            wantStop.set(true);
            System.out.println("Scheduler halting");
        }
        
    }

    /**
     * Determines if the scheduler is currently running. 
     * Will return true untill the scheduler has finished cleanup
     * @returns the current running state of the scheduler
     */
    public static boolean isRunning(){
        return isRunning.get();
    }

    /**
     * Adds an action to the scheduler to be started on the next iteration.
     * Actions can be scheduled at any time.
     * @param action the action to be scheduled
     * @param timeout_ms how long the action should run for (-1 if it should not time out)
     */
    public static void scheduleAction(Action action, long timeout_ms){
        if (action == null){
            //a null action has been passed
            System.out.println("Attempted to schedule a null action");
        } else {
            scheduled.add(new ScheduledAction(action, timeout_ms));
        }     
    }

    /**
     * Registers a trigger with the scheduler to detect if the trigger condition is met.
     * If the trigger condition is met, the associated action is started.
     * These should probably be intialized before the scheduler is started to work properly.
     * @param trigger the trigger to warch in the scheduler
     */
    public static void registerTrigger(Trigger trigger){
        triggers.add(trigger);
    }

}
