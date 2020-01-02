package frc.robot.actions.driveactions;

import frc.lib.statemachine.Action;
import frc.lib.util.DriveSignal;
import frc.robot.subsystems.Drive;

public class GyroLock extends Action {

    @Override
    public void onStart() {
        Drive.getInstance().setAnglePidLoop(DriveSignal.NEUTRAL,
         Drive.getInstance().getHeading().getDegrees());
    }

    @Override
    public void onLoop() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void onStop() {
        Drive.getInstance().setOpenLoop(DriveSignal.NEUTRAL);
    }
}
