package frc.lib.statemachine;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

public class StateMachine {

    private static final AtomicInteger state = new AtomicInteger(-1);
    private static final AtomicBoolean wantStop = new AtomicBoolean(true),
     stateLock = new AtomicBoolean(false);
    private volatile static StateMachineDescriptor internalDescriptor;
    private volatile static ConcurrentLinkedQueue<ActionGroup> queuedStates;
    private volatile static ActionGroup currentState;
    private volatile static double t_start;
    private static final double delay = 0.020;


    private static final Runnable Man = () -> {
        try {
            //state goes to 0 to start
            queuedStates = internalDescriptor.getStates();
            state.set(0);
            SmartDashboard.putNumber("StateMachine/state", state.get());

            if (queuedStates == null) {
                //a null state queue has been passed
                //need to set conditions and exit major loop
                state.set(-2);
                SmartDashboard.putNumber("StateMachine/state", state.get());
            } else {
                while (!queuedStates.isEmpty() && !wantStop.get()) {
                    // pull the next element from the queue and run the state
                    SmartDashboard.putNumber("StateMachine/state", state.get());
                    currentState = queuedStates.poll();
                    currentState.onStart();

                    //wait for the state to complete exectuing
                    while (!currentState.isFinished() && !wantStop.get()) {
                        t_start = Timer.getFPGATimestamp();
                        currentState.onLoop();
                        Timer.delay(delay - (Timer.getFPGATimestamp() - t_start));
                    }

                    //when complete stop the state and increment the state counter
                    currentState.onStop();
                    state.getAndAdd(1);
                }
            }
            
        }catch (Exception e){
            //error condition, a exception was thrown during state machine execution
            System.out.println(e.getMessage());
            state.set(-3);
            SmartDashboard.putNumber("StateMachine/state", state.get());

        } finally{

            //either route, the state lock has to be un-set for the state machine to be re-used
            stateLock.set(false);
        }
    };

    /**
     * Creates a single active state machine thread to execute the passed descriptor.
     * It will not start a new descriptor is the machine is currently active.
     * @param descriptor the state machine will be engaged to run
     * @return status of the passed descriptor. If true, the descriptor was started sucessfully
     */
    public static boolean runMachine(StateMachineDescriptor descriptor) {
        //if the machine is currenly running it must be shut down independently in order to start a new descriptor
        if(stateLock.get()) return false;

        //entered a resettable state
        //reset the state booleans
        SmartDashboard.putNumber("StateMachine/state", -1);
        stateLock.set(true);
        wantStop.set(false);

        //move the new state machine in
        internalDescriptor = descriptor;

        //start the new state machine thread
        Thread thread = new Thread(Man);
        thread.start();

        return true;
    }

    /**
     * @return if the state machine is currently active
     */
    public static boolean isRunning(){
        return stateLock.get();
    }

    /**
     * asserts the shutdown state of the state machine. The machine will stop within the next iteration of the minor loop (the value of delay)
     */
    public static void assertStop(){
        SmartDashboard.putNumber("StateMachine/state", -1);
        if(!wantStop.get()){
            wantStop.set(true);
            System.out.println("State Machine Halting");
        }
        
    }


}
