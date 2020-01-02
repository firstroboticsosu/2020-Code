package frc.robot;

import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.lib.loops.Looper;
import frc.lib.util.ReflectingLogger;
import frc.robot.subsystems.Subsystem;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;

public class SubsystemManager implements ILooper {

    private final List<Subsystem> mAllSubsystems;
    private List<Loop> mLoops = new ArrayList<>();
    private ReflectingLogger<Subsystem.PeriodicIO> logger;

    /**
     * a manager class to handle all of the individual subsystems
     * @param allSubsystems -- a list of subsystems in order of importance.
     * Dependent subsystems should be reset before their parents
     * @param ignoreLogger -- ignores the initialization of the logger
     * and any subsequent errors it may generate
     */
    public SubsystemManager(List<Subsystem> allSubsystems, boolean ignoreLogger){
        mAllSubsystems = allSubsystems;

        //get all subsystems to log from
        final List<Subsystem.PeriodicIO> allToLog = new ArrayList<>();
        mAllSubsystems.forEach((s) -> allToLog.add(s.getLogger()));

        try{
            //create reflecting logger
            logger = new ReflectingLogger<>(allToLog);
        } catch (Exception e){
            // show logger failed to init
            DriverStation.reportError("Logger unable to start", e.getStackTrace());

            //throw the runtime error only if turned on via argument
            if(!ignoreLogger) throw new RuntimeException("Error instantiating the logger");
        }

    }

    /**
     * Runs a pass of the reflection based logger over all substems
     */
    public void logTelemetry(){
        //make sure logger is properly initialized
        if(logger != null) {
            // create current list of subsystem IO
            final List<Subsystem.PeriodicIO> allToLog = new ArrayList<>();
            mAllSubsystems.forEach((s) -> allToLog.add(s.getLogger()));

            //update the logger from the current form of the list
            logger.update(allToLog);
        }
    }

    /**
     * Runs the output telemetry method on all subsystems to communicate data
     */
    public void outputTelemetry(){
        mAllSubsystems.forEach(Subsystem::outputTelemetry);
    }

    /**
     * Calls the onStop method for all subsystems to put the robot into
     * an open loop control mode
     */
    public void onStop(){
        mAllSubsystems.forEach(Subsystem::onStop);
    }

    /**
     * Resets all subsystems in the order that they were registered
     * with the subsystem manager. If the order is incorrect determinism
     * during the reset may be lost
     */
    public void resetAllSubsystems(){
        mAllSubsystems.forEach(Subsystem::reset);
    }

    private class EnabledLoop implements Loop {

        @Override
        public void onStart(double timestamp) {
            for (Loop l : mLoops) {
                l.onStart(timestamp);
            }
        }

        @Override
        public void onLoop(double timestamp) {
            for (Subsystem s : mAllSubsystems) {
                s.readPeriodicInputs();
            }
            for (Loop l : mLoops) {
                l.onLoop(timestamp);
            }
            for (Subsystem s : mAllSubsystems) {
                s.writePeriodicOutputs();
            }

            //run logging pass
            logTelemetry();
        }

        @Override
        public void onStop(double timestamp) {
            for (Loop l : mLoops) {
                l.onStop(timestamp);
            }
        }
    }

    private class DisabledLoop implements Loop {

        @Override
        public void onStart(double timestamp) {

        }

        @Override
        public void onLoop(double timestamp) {
            for (Subsystem s : mAllSubsystems) {
                s.readPeriodicInputs();
            }
            for (Subsystem s : mAllSubsystems) {
                s.writePeriodicOutputs();
            }
        }

        @Override
        public void onStop(double timestamp) {
        }
    }

    public void registerEnabledLoops(Looper enabledLooper) {
        mAllSubsystems.forEach((s) -> s.registerEnabledLoops(this));
        enabledLooper.register(new EnabledLoop());
    }

    public void registerDisabledLoops(Looper disabledLooper) {
        disabledLooper.register(new DisabledLoop());
    }

    @Override
    public void register(Loop loop) {
        mLoops.add(loop);
    }

}
