package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Rotation2d;
import frc.lib.geometry.Twist2d;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.lib.math.InterpolatingDouble;
import frc.lib.math.InterpolatingTreeMap;
import frc.robot.Kinematics;

import java.util.Map;


public class PoseEstimator extends Subsystem {

    private static PoseEstimator m_instance = new PoseEstimator();

    private static final int observation_buffer_size_ = 100;

    private PoseIO periodicIO;
    public InterpolatingTreeMap<InterpolatingDouble, Pose2d> field_to_vehicle_;


    private Loop mLoop = new Loop(){

        @Override
        public void onStart(double timestamp) {
            periodicIO.distance_driven_= 0.0;
            periodicIO.left_encoder_prev_distance_ = Drive.getInstance().getLeftEncoderDistance();
            periodicIO.right_encoder_prev_distance_ = Drive.getInstance().getRightEncoderDistance();
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (this){
                final Rotation2d gyro_angle = Drive.getInstance().getHeading();
                final double left_distance = Drive.getInstance().getLeftEncoderDistance();
                final double right_distance = Drive.getInstance().getRightEncoderDistance();
                final double delta_left = left_distance - periodicIO.left_encoder_prev_distance_;
                final double delta_right = right_distance - periodicIO.right_encoder_prev_distance_;
                final Twist2d odometry_velocity = generateOdometryFromSensors(delta_left, delta_right, gyro_angle);
                addObservations(timestamp, Kinematics.integrateForwardKinematics(getLatestFieldToVehicle().getValue(), odometry_velocity),
                    odometry_velocity);
                periodicIO.left_encoder_prev_distance_ = left_distance;
                periodicIO.right_encoder_prev_distance_ = right_distance;
                outputTelemetry();
            }
        }

        @Override
        public void onStop(double timestamp) {

        }
    };

    public static PoseEstimator getInstance(){
        return m_instance;
    }

    private PoseEstimator(){
        reset(0, Pose2d.identity());
    }

    public void reset(double start_time, Pose2d initial_field_to_vehicle){
        periodicIO = new PoseIO();
        field_to_vehicle_ = new InterpolatingTreeMap<>(observation_buffer_size_);
        field_to_vehicle_.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
        periodicIO.distance_driven_ = 0.0;
        periodicIO.vehicle_velocity_predicted_ = Twist2d.identity();
    }

    public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestFieldToVehicle() {
        return field_to_vehicle_.lastEntry();
    }

    public synchronized Pose2d getFieldToVehicle(double timestamp) {
        return field_to_vehicle_.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Twist2d generateOdometryFromSensors(double left_encoder_delta_distance, double
            right_encoder_delta_distance, Rotation2d current_gyro_angle) {
        final Pose2d last_measurement = getLatestFieldToVehicle().getValue();
        final Twist2d delta = Kinematics.forwardKinematics(last_measurement.getRotation(),
                left_encoder_delta_distance, right_encoder_delta_distance,
                current_gyro_angle);
        periodicIO.distance_driven_ += delta.dx; 

        return delta;
    }

    public synchronized void addObservations(double timestamp, Pose2d observation, Twist2d velocity) {
        field_to_vehicle_.put(new InterpolatingDouble(timestamp), observation);
        periodicIO.vehicle_velocity_predicted_ = velocity;
    }

    public synchronized double getDistanceDriven() {
        return periodicIO.distance_driven_;
    }

    public synchronized void resetDistanceDriven() {
        periodicIO.distance_driven_ = 0.0;
    }

    public synchronized Twist2d getPredictedVelocity() {
        return periodicIO.vehicle_velocity_predicted_;
    }

    @Override
    public void readPeriodicInputs() {

    }

    @Override
    public void writePeriodicOutputs() {

    }

    @Override
    public void outputTelemetry() {
        periodicIO.current = getLatestFieldToVehicle().getValue();
        SmartDashboard.putNumber("Drive/Pose/X", periodicIO.current.getTranslation().x());
        SmartDashboard.putNumber("Drive/Pose/Y", periodicIO.current.getTranslation().y());
        SmartDashboard.putNumber("Drive/Pose/Theta", (periodicIO.current.getRotation().getDegrees()+360)%360);
    }

    @Override
    public void reset() {
        reset(Timer.getFPGATimestamp(), Pose2d.identity());
    }

    @Override
    public void registerEnabledLoops(ILooper looper){
        looper.register(mLoop);
    }

    public class PoseIO extends PeriodicIO{
        public Pose2d current;
        public double distance_driven_= 0.0;
        public double left_encoder_prev_distance_ = 0.0;
        public double right_encoder_prev_distance_ = 0.0;
        public Twist2d vehicle_velocity_predicted_;

    }

    @Override
    public void onStop() {
        // No-op
    }

}
