package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.drivers.PIDF;
import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Rotation2d;
import frc.lib.geometry.Translation2d;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.lib.path.AdaptivePurePursuitController;
import frc.lib.path.Lookahead;
import frc.lib.path.Path;
import frc.lib.util.DriveSignal;
import frc.lib.util.HIDHelper;
import frc.lib.util.Util;
import frc.robot.Constants;
import frc.robot.Kinematics;

public class Drive extends Subsystem {

    //construct one and only 1 instance of this class
    private static Drive m_DriveInstance = new Drive();

    public static Drive getInstance() {
        return m_DriveInstance;
    }

    //used internally for data
    private DriveControlState mDriveControlState = DriveControlState.OPEN_LOOP;
    private DriveIO periodicIO;

    // Hardware
    private PigeonIMU pigeonIMU;
    private TalonSRX driveFrontLeft, driveFrontRight; 
    private VictorSPX driveRearLeft, driveRearRight;
    
    // Controllers
    private AdaptivePurePursuitController mPathFollower;
    private Path currentPath;
    private PIDF angleController;

    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
            }
    
            @Override
            public void onLoop(double timestamp) {
                synchronized (Drive.this) {
                    if (Constants.ENABLE_MP_TEST_MODE && DriverStation.getInstance().isTest()) {
                        mDriveControlState = DriveControlState.PROFILING_TEST;
                    }
    
                    switch (mDriveControlState) {
                        case PATH_FOLLOWING:
                            updatePathFollower(timestamp);
                            break;
    
                        case PROFILING_TEST:
                        if (DriverStation.getInstance().isTest()) {
                            if (Constants.RAMPUP) {
                                periodicIO.left_demand = (periodicIO.ramp_Up_Counter * .0025 + .01);
                                periodicIO.right_demand = (periodicIO.ramp_Up_Counter * .0025 + .01);
                                periodicIO.ramp_Up_Counter++;
                            } else {
                                periodicIO.left_demand = radiansPerSecondToTicksPer100ms(inchesPerSecondToRadiansPerSecond(Constants.MP_TEST_SPEED));
                                periodicIO.right_demand = radiansPerSecondToTicksPer100ms(inchesPerSecondToRadiansPerSecond(Constants.MP_TEST_SPEED));
                            }
                        }
                        break;
    
                        case OPEN_LOOP:
                            setOpenLoop(arcadeDrive(periodicIO.operatorInput[1], periodicIO.operatorInput[2]).invert());
                            periodicIO.gyro_pid_angle = periodicIO.gyro_heading.getDegrees();
                            break;
    
                        case ANGLE_PID:
                            final double zRotation = -angleController.update(periodicIO.gyro_heading.getDegrees());
                            setAnglePidLoop(arcadeDrive(periodicIO.operatorInput[1], zRotation).invert(), periodicIO.gyro_pid_angle);
                            break;
                
                        default:
                            System.out.println("Unexpected control state");
                    }
    
    
                }
            }
    
            @Override
            public void onStop(double timestamp) {
    
            }
        });
    }

    @Override
    public synchronized void readPeriodicInputs() {
        double prevLeftTicks = periodicIO.left_pos_ticks;
        double prevRightTicks = periodicIO.right_pos_ticks;

        periodicIO.operatorInput = HIDHelper.getAdjStick(Constants.MASTER_STICK);

        periodicIO.left_pos_ticks = driveFrontLeft.getSelectedSensorPosition(0);
        periodicIO.right_pos_ticks = driveFrontRight.getSelectedSensorPosition(0);
        periodicIO.left_velocity_ticks_per_100ms = driveFrontLeft.getSelectedSensorVelocity(0);
        periodicIO.right_velocity_ticks_per_100ms = driveFrontRight.getSelectedSensorVelocity(0);
        periodicIO.gyro_heading = Rotation2d.fromDegrees(pigeonIMU.getFusedHeading()).rotateBy(periodicIO.gyro_offset);
        periodicIO.left_error = driveFrontLeft.getClosedLoopError();
        periodicIO.right_error = driveFrontRight.getClosedLoopError();

        double deltaLeftTicks = ((periodicIO.left_pos_ticks - prevLeftTicks) / 4096.0) * Math.PI;
        periodicIO.left_distance += deltaLeftTicks * Constants.DRIVE_WHEEL_DIAMETER_INCHES;
        double deltaRightTicks = ((periodicIO.right_pos_ticks - prevRightTicks) / 4096.0) * Math.PI;
        periodicIO.right_distance += deltaRightTicks * Constants.DRIVE_WHEEL_DIAMETER_INCHES;

        periodicIO.left_voltage = driveFrontLeft.getMotorOutputVoltage();
        periodicIO.right_voltage = driveFrontRight.getMotorOutputVoltage();
        periodicIO.left_current = driveFrontLeft.getOutputCurrent(); //TODO investigate possible EMI noise
        periodicIO.right_current = driveFrontRight.getOutputCurrent(); //TODO investigate possible EMI noise
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mDriveControlState == DriveControlState.OPEN_LOOP || mDriveControlState == DriveControlState.ANGLE_PID || (mDriveControlState == DriveControlState.PROFILING_TEST && Constants.RAMPUP)) {
            driveFrontLeft.set(ControlMode.PercentOutput, periodicIO.left_demand);
            driveFrontRight.set(ControlMode.PercentOutput, periodicIO.right_demand);

        } else {
            driveFrontLeft.set(ControlMode.Velocity, periodicIO.left_demand, DemandType.ArbitraryFeedForward, (periodicIO.left_feedforward + Constants.DRIVE_LEFT_KD * periodicIO.left_accl / 1023.0));
            driveFrontRight.set(ControlMode.Velocity, periodicIO.right_demand, DemandType.ArbitraryFeedForward, (periodicIO.right_feedforward + Constants.DRIVE_RIGHT_KD * periodicIO.right_accl / 1023.0));
        }
    }

    private Drive() {
        driveFrontLeft = new TalonSRX(Constants.DRIVE_FRONT_LEFT_ID);
        driveRearLeft = new VictorSPX(Constants.DRIVE_BACK_LEFT_ID);
        driveFrontRight = new TalonSRX(Constants.DRIVE_FRONT_RIGHT_ID);
        driveRearRight = new VictorSPX(Constants.DRIVE_BACK_RIGHT_ID);
        pigeonIMU = new PigeonIMU(Constants.PIGEON_IMU_ID);
        angleController = new PIDF(Constants.ANGLE_KP, Constants.ANGLE_KI, Constants.ANGLE_KD, Constants.ANGLE_IMAX);
        configTalons();
        reset();

    }

    public synchronized Rotation2d getHeading() {
        return periodicIO.gyro_heading;
    }

    public synchronized void setHeading(Rotation2d heading) {
        System.out.println("SET HEADING: " + heading.getDegrees());
        periodicIO.gyro_offset = heading.rotateBy(Rotation2d.fromDegrees(pigeonIMU.getFusedHeading()).inverse());
        System.out.println("Gyro offset: " + periodicIO.gyro_offset.getDegrees());
        periodicIO.gyro_heading = heading;
    }

    public double getLeftEncoderRotations() {
        return periodicIO.left_pos_ticks / Constants.DRIVE_ENCODER_PPR;
    }

    public double getRightEncoderRotations() {
        return periodicIO.right_pos_ticks / Constants.DRIVE_ENCODER_PPR;
    }

    public double getLeftEncoderDistance() {
        return rotationsToInches(getLeftEncoderRotations());
    }

    public double getRightEncoderDistance() {
        return rotationsToInches(getRightEncoderRotations());
    }

    public double getLeftVelocityNativeUnits() {
        return periodicIO.left_velocity_ticks_per_100ms;
    }

    public double getRightVelocityNativeUnits() {
        return periodicIO.right_velocity_ticks_per_100ms;
    }

    public double getLeftLinearVelocity() {
        return rotationsToInches(getLeftVelocityNativeUnits() * 10.0 / Constants.DRIVE_ENCODER_PPR);
    }

    public double getRightLinearVelocity() {
        return rotationsToInches(getRightVelocityNativeUnits() * 10.0 / Constants.DRIVE_ENCODER_PPR);
    }

    public void reset() {
        periodicIO = new DriveIO();
        setHeading(Rotation2d.fromDegrees(0));
        resetEncoders();
        angleController.reset();
    }

    private void resetEncoders() {
        driveFrontRight.setSelectedSensorPosition(0, 0, 0);
        driveFrontLeft.setSelectedSensorPosition(0, 0, 0);
    }

    private void configTalons() {
        ErrorCode sensorPresent;
        sensorPresent = driveFrontLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100); //primary closed-loop, 100 ms timeout
        if (sensorPresent != ErrorCode.OK) {
            DriverStation.reportError("Could not detect left encoder: " + sensorPresent, false);
        }
        driveFrontLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100); //DO NOT FORGET THIS use 5ms packet time on feedback
        driveFrontLeft.setSensorPhase(true);
        driveFrontLeft.selectProfileSlot(0, 0);
        driveFrontLeft.config_kF(0, Constants.DRIVE_LEFT_KF, 0);
        driveFrontLeft.config_kP(0, Constants.DRIVE_LEFT_KP, 0);
        driveFrontLeft.config_kI(0, Constants.DRIVE_LEFT_KI, 0);
        driveFrontLeft.config_kD(0, Constants.DRIVE_LEFT_KD, 0);
        driveFrontLeft.config_IntegralZone(0, 300);
        driveFrontLeft.setInverted(false);
        driveFrontLeft.setNeutralMode(NeutralMode.Brake);
        driveFrontLeft.configVoltageCompSaturation(Constants.DRIVE_VCOMP);
        driveFrontLeft.enableVoltageCompensation(true);
        

        driveRearLeft.setInverted(false);
        driveRearLeft.setNeutralMode(NeutralMode.Brake);
        driveRearLeft.configVoltageCompSaturation(Constants.DRIVE_VCOMP);
        driveRearLeft.enableVoltageCompensation(true);
        driveRearLeft.follow(driveFrontLeft);


        sensorPresent = driveFrontRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100); //primary closed-loop, 100 ms timeout
        if (sensorPresent != ErrorCode.OK) {
            DriverStation.reportError("Could not detect right encoder: " + sensorPresent, false);
        }
        driveFrontRight.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100); //DO NOT FORGET THIS use 5ms packet time on feedback
        driveFrontRight.setSensorPhase(true);
        driveFrontRight.selectProfileSlot(0, 0);
        driveFrontRight.config_kF(0, Constants.DRIVE_RIGHT_KF, 0);
        driveFrontRight.config_kP(0, Constants.DRIVE_RIGHT_KP, 0);
        driveFrontRight.config_kI(0, Constants.DRIVE_RIGHT_KI, 0);
        driveFrontRight.config_kD(0, Constants.DRIVE_RIGHT_KD, 0);
        driveFrontRight.config_IntegralZone(0, 300);
        driveFrontRight.setInverted(true);
        driveFrontRight.setNeutralMode(NeutralMode.Brake);
        driveFrontRight.configVoltageCompSaturation(Constants.DRIVE_VCOMP);
        driveFrontRight.enableVoltageCompensation(true);

        driveRearRight.setInverted(true);
        driveRearRight.setNeutralMode(NeutralMode.Brake);
        driveRearRight.configVoltageCompSaturation(Constants.DRIVE_VCOMP);
        driveRearRight.enableVoltageCompensation(true);
        driveRearRight.follow(driveFrontRight);

    }

    /**
     * Configures the drivebase to drive a path. Used for autonomous driving
     *
     * @see Path
     */
    public synchronized void setWantDrivePath(Path path, boolean reversed) {
        if (path != currentPath || mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            PoseEstimator.getInstance().resetDistanceDriven();
            mPathFollower = new AdaptivePurePursuitController(currentPath, reversed, 
              new Lookahead(Constants.PATH_MIN_LOOK_AHEAD_DIST, Constants.PATH_MAX_LOOK_AHEAD_DIST,
              Constants.PATH_MIN_LOOK_AHEAD_VEL, Constants.PATH_MAX_LOOK_AHEAD_VEL));
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
        } else {
            setVelocity(new DriveSignal(0, 0), new DriveSignal(0, 0));
        }
    }

    public synchronized boolean isDoneWithPath() {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            return mPathFollower.isFinished();
        } else {
            System.out.println("Robot is not in path following mode");
            return true;
        }
    }

    public synchronized void forceDoneWithPath() {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            setOpenLoop(DriveSignal.NEUTRAL);
        } else {
            System.out.println("Robot is not in path following mode");
        }
    }

    private void updatePathFollower(double timestamp) {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING) {
            Pose2d field_to_vehicle = PoseEstimator.getInstance().getLatestFieldToVehicle().getValue();
            AdaptivePurePursuitController.Command command = mPathFollower.update(field_to_vehicle);
            if (!mPathFollower.isFinished()) {
                periodicIO.pathLengthRemaining = command.remaining_path_length;
                periodicIO.lookaheadPt = command.lookahead_point;
                periodicIO.crossTrackError = command.cross_track_error;
                DriveSignal setpoint = Kinematics.inverseKinematics(command.delta);

                // Scale the command to respect the max velocity limits
                double max_vel = 0.0;
                max_vel = Math.max(max_vel, Math.abs(setpoint.getLeft()));
                max_vel = Math.max(max_vel, Math.abs(setpoint.getRight()));
                if (max_vel > Constants.PATH_MAX_VEL) {
                    double scaling = Constants.PATH_MAX_VEL / max_vel;
                    setpoint = new DriveSignal(setpoint.getLeft() * scaling, setpoint.getRight() * scaling);
                }

                //scale to respect velocity inputs
                setpoint = new DriveSignal(setpoint.getLeft(), setpoint.getRight());
                setVelocity(setpoint, DriveSignal.NEUTRAL);
            }
        } else {
            DriverStation.reportError("drive is not in path following state", false);
        }
    }

    public synchronized boolean hasPassedMarker(String marker) {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
            return mPathFollower.hasPassedMarker(marker);
        } else {
            System.out.println("Robot is not in path following mode");
            return false;
        }
    }

    /**
     * Configure talons for open loop control
     * @param signal input to drive train
     */
    public synchronized void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            System.out.println("Switching to open loop");
            driveFrontLeft.set(ControlMode.PercentOutput, 0);
            driveFrontRight.set(ControlMode.PercentOutput, 0);
            angleController.disable();
            mDriveControlState = DriveControlState.OPEN_LOOP;
        }
        periodicIO.left_demand = signal.getLeft();
        periodicIO.right_demand = signal.getRight();
    }

    /**
     * Configure talons for Angle PID control
     * @param signal input to drive train
     * @param angle target for the PID controller 
     */
    public synchronized void setAnglePidLoop(DriveSignal signal, double angle) {
        if (mDriveControlState != DriveControlState.ANGLE_PID) {
            System.out.println("Switching to angle control");
            driveFrontLeft.set(ControlMode.PercentOutput, 0);
            driveFrontRight.set(ControlMode.PercentOutput, 0);
            angleController.enable();
            mDriveControlState = DriveControlState.ANGLE_PID;
        }
        
        // Only update the setpoint if the value is not the current setpoint
        if(!Util.epsilonEquals(angle, periodicIO.gyro_pid_angle)){
            periodicIO.gyro_pid_angle = angle;
            angleController.setPoint(angle);
        }

        periodicIO.left_demand = signal.getLeft();
        periodicIO.right_demand = signal.getRight();
    }

    /**
     * Configures talons for velocity control
     */
    public synchronized void setVelocity(DriveSignal signal, DriveSignal feedforward) {
        if (mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            System.out.println("Switching to velocity control");
            driveFrontLeft.set(ControlMode.Velocity, 0);
            driveFrontRight.set(ControlMode.Velocity, 0);
            angleController.disable();
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
        }
        periodicIO.left_demand = signal.getLeft();
        periodicIO.right_demand = signal.getRight();
        periodicIO.left_feedforward = feedforward.getLeft();
        periodicIO.right_feedforward = feedforward.getRight();
    }

    @Override
    public PeriodicIO getLogger() {
        return periodicIO;
    }

    public void onStop(){
        forceDoneWithPath();
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    enum DriveControlState {
        OPEN_LOOP,
        PATH_FOLLOWING,
        PROFILING_TEST,
        ANGLE_PID,
        GYRO_LOCK;

        public String toString() {
            return name().charAt(0) + name().substring(1).toLowerCase();
        }
    }

    public void outputTelemetry() {
        SmartDashboard.putString("Drive/Drive State", mDriveControlState.toString());

        //SmartDashboard.putNumber("Drive/Error/X", periodicIO.error.getTranslation().x());
        //SmartDashboard.putNumber("Drive/Error/Y", periodicIO.error.getTranslation().y());
        //SmartDashboard.putNumber("Drive/Error/Theta", periodicIO.error.getRotation().getDegrees());
        //SmartDashboard.putNumber("Drive/Setpoint/X", periodicIO.path_setpoint.state().getTranslation().x());
        //SmartDashboard.putNumber("Drive/Setpoint/Y", periodicIO.path_setpoint.state().getTranslation().y());
        //SmartDashboard.putNumber("Drive/Setpoint/Theta", periodicIO.path_setpoint.state().getRotation().getDegrees());

        SmartDashboard.putNumber("Drive/Left/Demand", periodicIO.left_demand);
        SmartDashboard.putNumber("Drive/Left/Talon Velocity", periodicIO.left_velocity_ticks_per_100ms);
        //SmartDashboard.putNumber("Drive/Left/Talon Error", periodicIO.left_error);
        //SmartDashboard.putNumber("Drive/Left/Talon Voltage Out", driveFrontLeft.getMotorOutputVoltage());
        SmartDashboard.putNumber("Drive/Left/Encoder Counts", periodicIO.left_pos_ticks);
        //SmartDashboard.putNumber("Drive/Misc/Left FeedForward", periodicIO.left_feedforward);
        //SmartDashboard.putNumber("Drive/Misc/Left Acceleration", periodicIO.left_accl);


        SmartDashboard.putNumber("Drive/Right/Demand", periodicIO.right_demand);
        SmartDashboard.putNumber("Drive/Right/Talon Velocity", periodicIO.right_velocity_ticks_per_100ms);
        //SmartDashboard.putNumber("Drive/Right/Talon Error", periodicIO.right_error);
        //SmartDashboard.putNumber("Drive/Right/Talon Voltage Out", driveFrontRight.getMotorOutputVoltage());
        SmartDashboard.putNumber("Drive/Right/Encoder Counts", periodicIO.right_pos_ticks);
        //SmartDashboard.putNumber("Drive/Misc/Right FeedForward", periodicIO.right_feedforward);
        //SmartDashboard.putNumber("Drive/Misc/Right Acceleration", periodicIO.right_accl);
    }

    public class DriveIO extends PeriodicIO {
        // INPUTS
        // Left encoder values
        public int left_pos_ticks = 0;
        public int left_velocity_ticks_per_100ms = 0;
        public double left_error = 0.0;
        public double left_voltage = 0.0;
        public double left_current = 0.0;

        // Right encoder values
        public int right_pos_ticks = 0;
        public int right_velocity_ticks_per_100ms = 0;
        public double right_error = 0.0;
        public double right_voltage = 0.0;
        public double right_current = 0.0;

        // Gyro values
        public Rotation2d gyro_heading = Rotation2d.identity();
        public Rotation2d gyro_offset = Rotation2d.identity();

        // Pose system values
        public Translation2d lookaheadPt = Translation2d.identity();
        public double crossTrackError = 0.0;
        public double pathLengthRemaining = 0.0;

        // Gyro PID input
        public double gyro_pid_angle = 0.0;

        // Operator input
        public double[] operatorInput = {0, 0, 0};

        // OUTPUTS
        // Internal counters
        public double ramp_Up_Counter = 0;

        // Left talon output values
        public double left_accl = 0.0;
        public double left_demand = 0.0;
        public double left_distance = 0.0;
        public double left_feedforward = 0.0;

        // Right talon output values 
        public double right_accl = 0.0;
        public double right_demand = 0.0;
        public double right_distance = 0.0;
        public double right_feedforward = 0.0;
    }

    /**
     * internal methods beyond this point
     **/

    private static double rotationsToInches(double rotations) {
        return rotations * Math.PI * Constants.DRIVE_WHEEL_DIAMETER_INCHES;
    }

    private static double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }

    private static double inchesToRotations(double inches) {
        return inches / (Math.PI * Constants.DRIVE_WHEEL_DIAMETER_INCHES);
    }

    private static double inchesPerSecondToRpm(double inches_per_second) {
        return inchesToRotations(inches_per_second) * 60;
    }

    private static double radiansPerSecondToTicksPer100ms(double rad_s) {
        return rad_s / (Math.PI * 2.0) * 4096.0 / 10.0;
    }

    private static double inchesPerSecondToRadiansPerSecond(double in_sec) {
        return in_sec / (Constants.DRIVE_WHEEL_DIAMETER_INCHES * Math.PI) * 2 * Math.PI;
    }

    private static double rpmToTicksPer100ms(double rpm) {
        return ((rpm * 512.0) / 75.0);
    }

    private DriveSignal arcadeDrive(double xSpeed, double zRotation) {
        double leftMotorOutput;
        double rightMotorOutput;

        double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

        if (xSpeed >= 0.0) {
            // First quadrant, else second quadrant
            if (zRotation >= 0.0) {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
            } else {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
            }
        } else {
            // Third quadrant, else fourth quadrant
            if (zRotation >= 0.0) {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
            } else {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
            }
        }
        return new DriveSignal(rightMotorOutput, leftMotorOutput);
    }
}
