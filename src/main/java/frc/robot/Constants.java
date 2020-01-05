package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.lib.util.HIDHelper;

public class Constants {

    /**
     * device ID declarations ---------------------------------
     */

    //Talon IDs
    public static final int DRIVE_FRONT_LEFT_ID = 1;
    public static final int DRIVE_BACK_LEFT_ID = 3;
    public static final int DRIVE_FRONT_RIGHT_ID = 2;
    public static final int DRIVE_BACK_RIGHT_ID = 4;
    public static final int SPINNY_ID = 5;

    //Pigeon ID
    public static final int PIGEON_IMU_ID = 1;

    /**
     * Drivetrain tuned values --------------------------------
     */

    //Physical Constants
    public static final double DRIVE_WHEEL_DIAMETER_INCHES = 7.75; // 7 3/4
    public static final double DRIVE_WHEEL_RADIUS_INCHES = DRIVE_WHEEL_DIAMETER_INCHES / 2.0;
    public static final double TRACK_SCRUB_FACTOR = 2.38;  // determined 2.38
    public static final double TRACK_WIDTH_INCHES = 25;
    public static final double ROBOT_LINEAR_INERTIA = 30;  // kg roughly 60 lb
    public static final double ROBOT_ANGULAR_INERTIA = 8.0;  // kg m^2 TODO tune just a guess
    public static final double ROBOT_ANGULAR_DRAG = 20.0;  // N*m / (rad/sec) TODO tune just a guess
    public static final double ROBOT_MAX_VELOCITY = 72.0; // in/s
    public static final double ROBOT_MAX_ACCEL = 80.0; // in/s^2
    public static final double ROBOT_MAX_VOLTAGE = 11.0; // V


    public static final double PATH_KX = 4.0;  //
    public static final double PATH_LOOK_AHEAD_TIME = 0.4;  // seconds to look ahead along the path for steering
    public static final double PATH_MIN_LOOK_AHEAD_DIST = 12.0;  // inches
    public static final double PATH_MIN_LOOK_AHEAD_VEL = 24.0;  // inches/s
    public static final double PATH_MAX_LOOK_AHEAD_DIST = 60.0;  // inches
    public static final double PATH_MAX_LOOK_AHEAD_VEL = 96.0;  // inches/s
    public static final double PATH_MAX_VEL = 96.0;  // inches/s
    
    //Electrical Constants
    public static final double DRIVE_V_INTERCEPT = 1.04395;  // V     1.04395
    public static final double DRIVE_Kv = 0.32725;  // V per rad/s    0.32725
    public static final double DRIVE_Ka = 0.12545;  // V per rad/s^2  0.12545
    public static final double DRIVE_VCOMP = 11.0; //V                11.0
    public static final double DRIVE_ENCODER_PPR = 4096.0; //encoder counts per revolution

    //PID Constants
    public static final double ANGLE_KP = 0.04; // 0.065;
    public static final double ANGLE_KI = 0.0; // 0.00125;
    public static final double ANGLE_KD = 0.0; // 0.1
    public static final double ANGLE_IMAX = 1.0; // integral windup limit

    public static final double DRIVE_RIGHT_KP = 0.40; //0.40
    public static final double DRIVE_RIGHT_KI = 0.0;
    public static final double DRIVE_RIGHT_KD = 30.0; //30
    public static final double DRIVE_RIGHT_KF = 0.50; 

    public static final double DRIVE_LEFT_KP = 0.44; //0.44
    public static final double DRIVE_LEFT_KI = 0.0; 
    public static final double DRIVE_LEFT_KD = 30.0; //30
    public static final double DRIVE_LEFT_KF = 0.50;

    /**
     * Spinny Configuration --------------------------------------
     */
    // Percent speed at which to run for manual forward/backward
    public static final double PERCENT_MANUAL_FORWARD = .5;
    public static final double PERCENT_MANUAL_BACKWARD = -.5;

    // Center points for color inputs
    public static final int[] BLUE_IDEAL_COLOR_READINGS = {0, 255, 255};
    public static final int[] GREEN_IDEAL_COLOR_READINGS = {0, 255, 0};
    public static final int[] RED_IDEAL_COLOR_READINGS = {255, 0, 0};
    public static final int[] YELLOW_IDEAL_COLOR_READINGS = {255, 255, 0};

    // The maximum amount of deviation away from a color before it's not that color
    public static final int MAXIMUM_TOLERANCE = 100;

    /**
     * General Configuration --------------------------------------
     */

    //MP Test mode values
    public static final boolean ENABLE_MP_TEST_MODE = false; //enables motion profiling test across all modes
    public static final double MP_TEST_SPEED = 36; //in /s
    public static final boolean RAMPUP = false;

    //Stick Constants
    public static final Joystick MASTER = new Joystick(0);
    public static final HIDHelper.HIDConstants MASTER_STICK = new HIDHelper.HIDConstants(MASTER, 0.10, 1.00, 1.00, 0.50, 3);

    //Startup Constants
    public static final double LOOPER_DT = 0.01; //dt in seconds
	public static final double PATH_MAX_ACCEL = 0;
	
}



