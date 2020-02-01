package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Rotation2d;
import frc.lib.geometry.Twist2d;
import frc.lib.math.InterpolatingDouble;
import frc.lib.math.InterpolatingTreeMap;
import frc.robot.Kinematics;

import java.util.Map;


public class PoseEstimator {

    private static PoseEstimator m_instance = new PoseEstimator();

    private static final int observation_buffer_size_ = 100;

    public static Pose2d current;
    public static InterpolatingTreeMap<InterpolatingDouble, Pose2d> field_to_vehicle;
    public static double distance_driven = 0.0;
    public static double left_encoder_prev_distance = 0.0;
    public static double right_encoder_prev_distance = 0.0;

    public static void init() {
        distance_driven = 0.0;
        left_encoder_prev_distance = Drive.getLeftEncoderDistance();
        right_encoder_prev_distance = Drive.getRightEncoderDistance();
    }

    static void run() {
        final Rotation2d gyro_angle = Sensors.getHeading();
        final double left_distance = Drive.getLeftEncoderDistance();
        final double right_distance = Drive.getRightEncoderDistance();
        final double delta_left = left_distance - left_encoder_prev_distance;
        final double delta_right = right_distance - right_encoder_prev_distance;
        final Twist2d odometry_velocity = generateOdometryFromSensors(delta_left, delta_right, gyro_angle);
        addObservations(Timer.getFPGATimestamp(),
                Kinematics.integrateForwardKinematics(getLatestFieldToVehicle().getValue(), odometry_velocity));
        left_encoder_prev_distance = left_distance;
        right_encoder_prev_distance = right_distance;
        outputTelemetry();
    }

    public static PoseEstimator getInstance() {
        return m_instance;
    }

    private PoseEstimator() {
        reset(0, Pose2d.identity());
    }

    public static void reset(double start_time, Pose2d initial_field_to_vehicle) {
        field_to_vehicle = new InterpolatingTreeMap<>(observation_buffer_size_);
        field_to_vehicle.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
        distance_driven = 0.0;
    }

    public synchronized static Map.Entry<InterpolatingDouble, Pose2d> getLatestFieldToVehicle() {
        return field_to_vehicle.lastEntry();
    }

    public synchronized Pose2d getFieldToVehicle(double timestamp) {
        return field_to_vehicle.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized static Twist2d generateOdometryFromSensors(double left_encoder_delta_distance,
            double right_encoder_delta_distance, Rotation2d current_gyro_angle) {
        final Pose2d last_measurement = getLatestFieldToVehicle().getValue();
        final Twist2d delta = Kinematics.forwardKinematics(last_measurement.getRotation(), left_encoder_delta_distance,
                right_encoder_delta_distance, current_gyro_angle);
        distance_driven += delta.dx;
        return delta;
    }

    public synchronized static void addObservations(double timestamp, Pose2d observation) {
        field_to_vehicle.put(new InterpolatingDouble(timestamp), observation);
    }

    public double getDistanceDriven() {
        return distance_driven;
    }

    public static void outputTelemetry() {
        current = getLatestFieldToVehicle().getValue();
        SmartDashboard.putNumber("Drive/Pose/X", current.getTranslation().x());
        SmartDashboard.putNumber("Drive/Pose/Y", current.getTranslation().y());
        SmartDashboard.putNumber("Drive/Pose/Theta", (current.getRotation().getDegrees()+360)%360);
    }

    public static void reset() {
        reset(Timer.getFPGATimestamp(), Pose2d.identity());
    }

}
