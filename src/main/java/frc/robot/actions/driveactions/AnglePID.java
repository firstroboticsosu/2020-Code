package frc.robot.actions.driveactions;

import frc.lib.statemachine.Action;
import frc.lib.util.DriveSignal;
import frc.lib.util.Util;
import frc.robot.subsystems.Drive;

public class AnglePID extends Action {

    private double target;

    public AnglePID(double targetAngle){
        target = targetAngle;
    }

    @Override
    public void onStart() {
        Drive.getInstance().setAnglePidLoop(DriveSignal.NEUTRAL, target);
    }

    @Override
    public void onLoop() {

    }

    @Override
    public boolean isFinished() {
        //use a +- 1 degree epsilon
        return Util.epsilonEquals(target, Drive.getInstance().getHeading().getDegrees(), 1.0);
    }

    @Override
    public void onStop() {
        Drive.getInstance().setOpenLoop(DriveSignal.NEUTRAL);
    }

}
