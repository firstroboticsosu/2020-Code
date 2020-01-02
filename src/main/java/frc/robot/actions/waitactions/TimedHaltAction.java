package frc.robot.actions.waitactions;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.statemachine.Action;

public class TimedHaltAction extends Action {
    public TimedHaltAction(double timewanted) {
        timeWanted = timewanted;
    }

    private double timeStart, timeStop, timeWanted;
    private boolean done = false;

    @Override
    public void onStart() {
        timeStart = Timer.getFPGATimestamp();
    }

    @Override
    public void onLoop() {
        timeStop = Timer.getFPGATimestamp();
        if (timeWanted == timeStop - timeStart) {
            done = true;
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public void onStop() {
        done = false;
        timeStart = 0;
        timeStop = 0;
        timeWanted = 0;
    }
}
