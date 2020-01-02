package frc.robot.actions.driveactions;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.geometry.Pose2dWithCurvature;
import frc.lib.statemachine.Action;
import frc.lib.trajectory.TimedView;
import frc.lib.trajectory.Trajectory;
import frc.lib.trajectory.TrajectoryIterator;
import frc.lib.trajectory.timing.TimedState;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.PoseEstimator;


public class DriveTra extends Action {

    private static final Drive mDrive = Drive.getInstance();
    private static final PoseEstimator mRobotState = PoseEstimator.getInstance();
    private final TrajectoryIterator<TimedState<Pose2dWithCurvature>> mTra;
    private final boolean mResetPose;

    public DriveTra(Trajectory<TimedState<Pose2dWithCurvature>> Tra) {
        mTra = new TrajectoryIterator<>(new TimedView<>(Tra));
        mResetPose = false;
    }

    public DriveTra(Trajectory<TimedState<Pose2dWithCurvature>> Tra, boolean resetpos) {
        mTra = new TrajectoryIterator<>(new TimedView<>(Tra));
        mResetPose = resetpos;
    }

    @Override
    public void onStart() {
        System.out.println("Starting Tra");
        if (mResetPose) {
            mRobotState.reset(Timer.getFPGATimestamp(), mTra.getState().state().getPose());
        }
        mDrive.setTrajectory(mTra);
    }

    @Override
    public void onLoop() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void onStop() {

    }
}
