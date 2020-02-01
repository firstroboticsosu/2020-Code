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

public class Drive//holds the talons for driving and sets them up. Also does a little math
{
    enum DriveControlState {
        OPEN_LOOP("Open Loop"), PATH_FOLLOWING_CONTROL("Path following"), PROFILING_TEST("Profiling test");
        private String s;

        DriveControlState(String name){
            s = name;
        }

        @Override
        public String toString() {
            return s;
        }
    }
    static TalonSRX driveRightTalon;//1
    static TalonSRX driveLeftTalon;//2
    static VictorSPX driveRightVictor;//3
    static VictorSPX driveLeftVictor;//4
    static DriveControlState driveControlState;
    static int ramp_Up_Counter = 0;
    static void init()
    {
        driveRightTalon = new TalonSRX(Constants.DRIVE_RIGHT_TALON_ID);//init controllers
        driveLeftTalon = new TalonSRX(Constants.DRIVE_LEFT_TALON_ID);
        driveRightVictor = new VictorSPX(Constants.DRIVE_RIGHT_VICTOR_ID);
        driveLeftVictor = new VictorSPX(Constants.DRIVE_LEFT_VICTOR_ID);
        
        driveRightVictor.follow(driveRightTalon);//setup followers so we cant set the motors within a gearbox to go opposite directions 
        driveLeftVictor.follow(driveLeftTalon);

        // driveRightTalon.setInverted(true);//set one side inverted or else forward would spin the robot
        // driveRightVictor.setInverted(true);

        // driveRightTalon.setNeutralMode(NeutralMode.Brake);//easier to drive if on break imo
        // driveLeftVictor.setNeutralMode(NeutralMode.Brake);
        // driveRightTalon.setNeutralMode(NeutralMode.Brake);
        // driveRightVictor.setNeutralMode(NeutralMode.Brake);

        // driveRightTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);//config the feedback sensors, need it for auto
        // driveLeftTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        
        // driveLeftTalon.setSensorPhase(true);//makes sure the sensors positive corresponds to motor positive. need this or else PID wont work
        // driveRightTalon.setSensorPhase(true);

        
        // driveLeftTalon.config_kP(Constants.kPrimaryPIDSlot, Constants.drivekP);//setup some PID stuff
        // driveLeftTalon.config_kI(Constants.kPrimaryPIDSlot, Constants.drivekI);
        // driveLeftTalon.config_kD(Constants.kPrimaryPIDSlot, Constants.drivekD);
        
        // driveRightTalon.config_kP(Constants.kPrimaryPIDSlot, Constants.drivekP);//and for the other side
        // driveRightTalon.config_kI(Constants.kPrimaryPIDSlot, Constants.drivekI);
        // driveRightTalon.config_kD(Constants.kPrimaryPIDSlot, Constants.drivekD);

        // driveLeftTalon.configMotionProfileTrajectoryPeriod(Constants.pathPlannerTimeStepMs);//config the motion profile for auto to the time steps of PathPlanner
        // driveRightTalon.configMotionProfileTrajectoryPeriod(Constants.pathPlannerTimeStepMs);
        configTalons();
    }
    public static void resetEncoders()
    {
        Drive.driveRightTalon.setSelectedSensorPosition(0);
        Drive.driveLeftTalon.setSelectedSensorPosition(0);
    }
    public static int getLeftEncoderDistance()
    {
        return Drive.driveLeftTalon.getSelectedSensorPosition();
    }
    public static int getRightEncoderDistance()
    {
        return Drive.driveRightTalon.getSelectedSensorPosition();
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
    static void setVomp(boolean on)
    {
        driveLeftVictor.enableVoltageCompensation(on);
        driveLeftTalon.enableVoltageCompensation(on);
        driveRightTalon.enableVoltageCompensation(on);
        driveRightVictor.enableVoltageCompensation(on);
    }
    static void drive(Joystick stick)//single function to drive, called from telop
    {
        tank(stick.getRawAxis(1)+stick.getRawAxis(2), stick.getRawAxis(1)-stick.getRawAxis(2));
    }
    static void tank(double left, double right)//set motors on percent output specifically just in case the encoders failed
    {
        Drive.driveControlState = DriveControlState.OPEN_LOOP;
        driveLeftTalon.set(ControlMode.PercentOutput, left);
        driveRightTalon.set(ControlMode.PercentOutput, right);
    }
    public static void test() 
    {
        double left_demand = 0;
        double right_demand = 0;
        if (Constants.RAMP_UP)
        {
            left_demand = -(ramp_Up_Counter * .0025 + .01);
            right_demand = -(ramp_Up_Counter * .0025 + .01);
            ramp_Up_Counter++;
        } else {
            left_demand = Constants.MP_TEST_SPEED*Constants.kSensorUnitsPerMeter;
            right_demand = Constants.MP_TEST_SPEED*Constants.kSensorUnitsPerMeter;
        }
        driveLeftTalon.set(ControlMode.Velocity, left_demand);
        driveRightTalon.set(ControlMode.Velocity, right_demand);
    }
}