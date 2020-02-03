package frc.robot;


class Constants
{//file for all of our constants, more stuff could be moved here
	public static double kSensorUnitsPerMeter = 8459.75;//ticks per meter
	//data: 8438, 8493, 8475, 8433
	//determined by experimentation, could be determinded mathematically I guess
	public static int kPrimaryPIDSlot = 0;
	public static double drivekP = .1;
	public static double drivekI = 0;
	public static double drivekD = 1;
	public static int pathPlannerTimeStepMs = 10;
	public static int spin3ticks = 4096*64;



	public static final String kDefaultAuto = "Default";
	public static final String kBottom = "Bottom";
	public static final String kMiddle = "Middle";

	public static final int DRIVE_RIGHT_TALON_ID = 1;
	public static final int DRIVE_LEFT_TALON_ID = 2;
	public static final int DRIVE_RIGHT_VICTOR_ID = 3;
	public static final int DRIVE_LEFT_VICTOR_ID = 4;
	public static final int MECH_SPINNER_TALON_ID = 5;
	public static final int MECH_CLIMBER_TALON_ID = 6;
	public static final int MECH_HARVY_SPARK_ID = 9;
	public static final int MECH_HARVY_CLIMBER_ID = 10;
	public static final int MECH_SOLENOID_DOOR_FORWARD_ID = 0;
	public static final int MECH_SOLENOID_DOOR_BACKWARD_ID = 1;
	public static final int MECH_SOLENOID_COLOR_FORWARD_ID = 4;
	public static final int MECH_SOLENOID_COLOR_BACKWARD_ID = 5;
	public static final int PIDGEY_ID = 1;



	public static final double DRIVE_LEFT_KF = .3;
	public static final double DRIVE_LEFT_KP = .2;
	public static final double DRIVE_LEFT_KI = 0;
	public static final double DRIVE_LEFT_KD = 1;
	public static final double DRIVE_RIGHT_KF = .3;
	public static final double DRIVE_RIGHT_KP = .2;
	public static final double DRIVE_RIGHT_KI = 0;
	public static final double DRIVE_RIGHT_KD = 1;
	public static final double DRIVE_VCOMP = 10;



	public static final double DRIVE_WHEEL_TRACK_WIDTH_METERS = 0.635;
	public static final double TRACK_SCRUB_FACTOR = 2.0;
	public static final double PATH_FOLLOWING_MAX_ACCELERATION = 4.0;
    public static double DRIVETRAIN_UPDATE_RATE = 0.020;
	public static final double PATH_FOLLOWING_LOOKAHEAD = 1;
	public static final double PATH_FOLLOWING_MAX_VELOCITY = 0;
	public static final double WHEEL_DIAMETER = 0.1524;//meters
	public static final double MP_TEST_SPEED = 0.3;

	public static boolean PROFILE_TESTING = false;
	public static boolean RAMP_UP = false;
}