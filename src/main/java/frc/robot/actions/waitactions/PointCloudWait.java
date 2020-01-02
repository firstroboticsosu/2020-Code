package frc.robot.actions.waitactions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.geometry.Pose2d;
import frc.lib.statemachine.Action;
import frc.lib.util.Util;
import frc.robot.subsystems.PoseEstimator;

public class PointCloudWait extends Action {
    private boolean isX, isY, isTheta;
    private double epsilonX, epsilonY, epsilonTheta, X, Y, Theta;

    public PointCloudWait(Pose2d pose, double epsilonX, double epsilonY, double epsilonTheta) {
        Theta = pose.getRotation().getDegrees();
        X = pose.getTranslation().x();
        Y = pose.getTranslation().y();
        this.epsilonX = epsilonX;
        this.epsilonY = epsilonY;
        this.epsilonTheta = epsilonTheta;
    }

    @Override
    public void onStart() {
        isX = false;
        isY = false;
        isTheta = false;

    }

    public void onLoop() {
        double mX = PoseEstimator.getInstance().getLatestFieldToVehicle().getValue().getTranslation().x();
        double mY = PoseEstimator.getInstance().getLatestFieldToVehicle().getValue().getTranslation().y();
        double mTheta = PoseEstimator.getInstance().getLatestFieldToVehicle().getValue().getRotation().getDegrees();
        isX = Util.epsilonEquals(X, mX, epsilonX);
        isY = Util.epsilonEquals(Y, mY, epsilonY);
        isTheta = Util.epsilonEquals(Theta, mTheta, epsilonTheta);
        SmartDashboard.putBoolean("isX", isX);
        SmartDashboard.putBoolean("isY", isY);
        SmartDashboard.putBoolean("isTheta", isTheta);
    }

    public boolean isFinished() {
        return (isX && isY) && isTheta;
    }

    public void onStop() {

    }
}
