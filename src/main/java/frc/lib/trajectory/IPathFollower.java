package frc.lib.trajectory;

import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Twist2d;

public interface IPathFollower {
    public Twist2d steer(Pose2d current_pose);

    public boolean isDone();
}
