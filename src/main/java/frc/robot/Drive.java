package frc.robot;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.control.AdaptivePurePursuitController;
import frc.lib.control.Path;
import frc.lib.geometry.*;
import frc.lib.util.DriveSignal;

public class Drive// holds the talons for driving and sets them up. Also does a little math
{
    enum DriveControlState {
        OPEN_LOOP("Open Loop"), PATH_FOLLOWING_CONTROL("Path following"), PROFILING_TEST("Profiling test");

        private String s;

        DriveControlState(String name) {
            s = name;
        }

        @Override
        public String toString() {
            return s;
        }
    }

    static TalonSRX driveRightTalon;// 1
    static TalonSRX driveLeftTalon;// 2
    static VictorSPX driveRightVictor;// 3
    static VictorSPX driveLeftVictor;// 4
    static DriveControlState driveControlState;
    static int ramp_Up_Counter = 0;
    static AdaptivePurePursuitController pathFollowingController;

    public static void init() {
        driveRightTalon = new TalonSRX(Constants.DRIVE_RIGHT_TALON_ID);// init controllers
        driveLeftTalon = new TalonSRX(Constants.DRIVE_LEFT_TALON_ID);
        driveRightVictor = new VictorSPX(Constants.DRIVE_RIGHT_VICTOR_ID);
        driveLeftVictor = new VictorSPX(Constants.DRIVE_LEFT_VICTOR_ID);

        configTalons();
    }

    public static void resetEncoders() {
        driveRightTalon.setSelectedSensorPosition(0);
        driveLeftTalon.setSelectedSensorPosition(0);
    }

    public static double getLeftEncoderDistance() {
        return ticksToMeters(driveLeftTalon.getSelectedSensorPosition());
    }

    public static double getRightEncoderDistance() {
        return ticksToMeters(driveRightTalon.getSelectedSensorPosition());
    }

    private static void configTalons() {
        ErrorCode sensorPresent = driveRightTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
                0, 100); // primary closed-loop, 100 ms timeout
        if (sensorPresent != ErrorCode.OK) {
            DriverStation.reportError("Could not detect left encoder: " + sensorPresent, false);
        }
        driveLeftTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100); // DO NOT FORGET THIS use
                                                                                             // 5ms packet time on
                                                                                             // feedback
        driveLeftTalon.setSensorPhase(true);
        driveLeftTalon.selectProfileSlot(0, 0);
        driveLeftTalon.config_kF(0, Constants.DRIVE_LEFT_KF, 0);
        driveLeftTalon.config_kP(0, Constants.DRIVE_LEFT_KP, 0);
        driveLeftTalon.config_kI(0, Constants.DRIVE_LEFT_KI, 0);
        driveLeftTalon.config_kD(0, Constants.DRIVE_LEFT_KD, 0);
        driveLeftTalon.config_IntegralZone(0, 300);
        driveLeftTalon.setInverted(false);
        driveLeftTalon.setNeutralMode(NeutralMode.Brake);
        driveLeftTalon.configVoltageCompSaturation(Constants.DRIVE_VCOMP);
        driveLeftTalon.enableVoltageCompensation(true);

        driveLeftVictor.setInverted(false);
        driveLeftVictor.setNeutralMode(NeutralMode.Brake);
        driveLeftVictor.configVoltageCompSaturation(Constants.DRIVE_VCOMP);
        driveLeftVictor.enableVoltageCompensation(true);
        driveLeftVictor.follow(driveLeftTalon);

        sensorPresent = driveLeftTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100); // primary
                                                                                                                      // closed-loop,
                                                                                                                      // 100
                                                                                                                      // ms
                                                                                                                      // timeout
        if (sensorPresent != ErrorCode.OK) {
            DriverStation.reportError("Could not detect right encoder: " + sensorPresent, false);
        }
        driveRightTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100); // DO NOT FORGET THIS use
                                                                                              // 5ms packet time on
                                                                                              // feedback
        driveRightTalon.setSensorPhase(true);
        driveRightTalon.selectProfileSlot(0, 0);
        driveRightTalon.config_kF(0, Constants.DRIVE_RIGHT_KF, 0);
        driveRightTalon.config_kP(0, Constants.DRIVE_RIGHT_KP, 0);
        driveRightTalon.config_kI(0, Constants.DRIVE_RIGHT_KI, 0);
        driveRightTalon.config_kD(0, Constants.DRIVE_RIGHT_KD, 0);
        driveRightTalon.config_IntegralZone(0, 300);
        driveRightTalon.setInverted(true);
        driveRightTalon.setNeutralMode(NeutralMode.Brake);
        driveRightTalon.configVoltageCompSaturation(Constants.DRIVE_VCOMP);
        driveRightTalon.enableVoltageCompensation(true);

        driveRightVictor.setInverted(true);
        driveRightVictor.setNeutralMode(NeutralMode.Brake);
        driveRightVictor.configVoltageCompSaturation(Constants.DRIVE_VCOMP);
        driveRightVictor.follow(driveRightTalon);

    }

    public static void setVcomp(boolean on) {
        driveLeftVictor.enableVoltageCompensation(on);
        driveLeftTalon.enableVoltageCompensation(on);
        driveRightTalon.enableVoltageCompensation(on);
        driveRightVictor.enableVoltageCompensation(on);
    }

    public static void drive(Joystick stick) {
        tank(stick.getRawAxis(1) + stick.getRawAxis(2), stick.getRawAxis(1) - stick.getRawAxis(2));
    }

    public static void tank(double left, double right) {
        Drive.driveControlState = DriveControlState.OPEN_LOOP;
        driveLeftTalon.set(ControlMode.PercentOutput, left);
        driveRightTalon.set(ControlMode.PercentOutput, right);
    }

    public static void test() {
        double left_demand = 0;
        double right_demand = 0;
        if (Constants.RAMP_UP) {
            left_demand = -(ramp_Up_Counter * .0025 + .01);
            right_demand = -(ramp_Up_Counter * .0025 + .01);
            ramp_Up_Counter++;
        } else {
            left_demand = Constants.MP_TEST_SPEED * Constants.kSensorUnitsPerMeter;
            right_demand = Constants.MP_TEST_SPEED * Constants.kSensorUnitsPerMeter;
        }
        setVelocity(new DriveSignal(left_demand, right_demand));
    }

    public static void reset() {
        pathFollowingController = null;
    }

    public static void followPath(Path path, boolean reversed) {
        pathFollowingController = new AdaptivePurePursuitController(Constants.PATH_FOLLOWING_LOOKAHEAD,
                Constants.PATH_FOLLOWING_MAX_ACCELERATION, Constants.DRIVETRAIN_UPDATE_RATE, path, reversed, 1);
        Drive.driveControlState = DriveControlState.PATH_FOLLOWING_CONTROL;
        updatePathFollower();
    }

    public static double ticksToMeters(double ticks){
        return ticks / Constants.kSensorUnitsPerMeter;
    }

    public static double metersToTicks(double meters){
        return meters * Constants.kSensorUnitsPerMeter;
    }

    public static double metersToRotations(double meters) {
        return meters / (Constants.WHEEL_DIAMETER * Math.PI);
    }

    public static double metersPerSecondToRpm(double mps) {
        return metersToRotations(mps) * 60;
    }

    public static double RPMToUnitsPer100Ms(double RPM){
        return (RPM * 512) / 75.0;
    }

    public static void updatePathFollower() {
        Pose2d robot_pose = PoseEstimator.getLatestFieldToVehicle().getValue();
        Twist2d command = pathFollowingController.update(robot_pose, Timer.getFPGATimestamp());
        DriveSignal setpoint = Kinematics.inverseKinematics(command);

        // Scale the command to respect the max velocity limits
        double max_vel = 0.0;
        max_vel = Math.max(max_vel, Math.abs(setpoint.getLeft()));
        max_vel = Math.max(max_vel, Math.abs(setpoint.getRight()));
        if (max_vel > Constants.PATH_FOLLOWING_MAX_VELOCITY) {
            double scaling = Constants.PATH_FOLLOWING_MAX_VELOCITY / max_vel;
            setpoint = new DriveSignal(setpoint.getLeft() * scaling, setpoint.getRight() * scaling);
        }
        setpoint = new DriveSignal(RPMToUnitsPer100Ms(metersPerSecondToRpm(setpoint.getLeft())),
         RPMToUnitsPer100Ms(metersPerSecondToRpm(setpoint.getRight())));
        setVelocity(setpoint);
        // driveTank(metersPerSecondToRpm(setpoint.getLeft()),
        // metersPerSecondToRpm(setpoint.getRight()));
    }

    public static boolean isFinishedPath() {
        return (Drive.driveControlState == DriveControlState.PATH_FOLLOWING_CONTROL && pathFollowingController.isDone())
                || Drive.driveControlState != DriveControlState.PATH_FOLLOWING_CONTROL;
    }

    public static void setVelocity(DriveSignal signal) {
        if (Drive.driveControlState != DriveControlState.PATH_FOLLOWING_CONTROL) {
            System.out.println("Switching to velocity control");
            Drive.driveLeftTalon.set(ControlMode.Velocity, 0);
            Drive.driveRightTalon.set(ControlMode.Velocity, 0);
            Drive.driveControlState = DriveControlState.PATH_FOLLOWING_CONTROL;
        }
        SmartDashboard.putNumber("left target", signal.getLeft());
        SmartDashboard.putNumber("right target", signal.getRight());
        SmartDashboard.putNumber("left val", driveLeftTalon.getSelectedSensorVelocity());
        SmartDashboard.putNumber("right val", driveRightTalon.getSelectedSensorVelocity());
        Drive.driveLeftTalon.set(ControlMode.Velocity, signal.getLeft());
        Drive.driveRightTalon.set(ControlMode.Velocity, signal.getRight());
    }
}