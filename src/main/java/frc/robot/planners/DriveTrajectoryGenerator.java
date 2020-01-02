package frc.robot.planners;

import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Pose2dWithCurvature;
import frc.lib.geometry.Rotation2d;
import frc.lib.trajectory.Trajectory;
import frc.lib.trajectory.timing.CentripetalAccelerationConstraint;
import frc.lib.trajectory.timing.TimedState;
import frc.lib.trajectory.timing.TimingConstraint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class DriveTrajectoryGenerator {
    private static final DriveTrajectoryGenerator m_instance = new DriveTrajectoryGenerator();
    private final DriveMotionPlanner DMP;

    private DriveTrajectoryGenerator() {
        DMP = new DriveMotionPlanner();
    }

    public static DriveTrajectoryGenerator getInstance() {
        return m_instance;
    }

    /**
     * generates a trajectory from a list of pose2d's
     * @param reversed should the robot be running this in reverse
     * @param waypoints list of waypoints t execute
     * @param constraints list of timing constriants to apply to the trajectory
     * @param max_vel maximum velocity that the robot will travel at
     * @param max_accel maximum acceleration the robot is allowed to use while accelerating
     * @param max_voltage maximum voltage the controllers are allowed to apply
     * @returns the timed trajectory for use by the drivetrain
     */
    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage) {
        return DMP.generateTrajectory(reversed, waypoints, constraints, 0.0, 0.0, max_vel, max_accel, max_voltage);
    }

    /**
     * get the ten foot test trajectory
     * @return 10 foot forward test trajectory
     */
    public Trajectory<TimedState<Pose2dWithCurvature>> getTenFeet() {
        List<Pose2d> points = new ArrayList<>();
        points.add(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
        points.add(new Pose2d(120, 0, Rotation2d.fromDegrees(0)));
        return generateTrajectory(false, points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 48.0, 50, 10.0);
    }

    /**
     * gets a trajectory with a slight curve to the left in it. runs at 36 in/s
     * @return 10 foot forward trajectory with slight curve
     */
    public Trajectory<TimedState<Pose2dWithCurvature>> getTestCurve(){
        List<Pose2d> points = new ArrayList<>();
        points.add(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
        points.add(new Pose2d(60, 12, Rotation2d.fromDegrees(0)));
        points.add(new Pose2d(120, 0, Rotation2d.fromDegrees(0)));
        return generateTrajectory(false, points, Arrays.asList(new CentripetalAccelerationConstraint(60)), 36.0, 50, 10.0);
    }
}