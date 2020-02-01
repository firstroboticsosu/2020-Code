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
}