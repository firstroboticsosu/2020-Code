package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Joystick;

public class Drive//holds the talons for driving and sets them up. Also does a little math
{
    static WPI_TalonSRX driveRightTalon;//1
    static WPI_TalonSRX driveLeftTalon;//2
    static WPI_VictorSPX driveRightVictor;//3
    static WPI_VictorSPX driveLeftVictor;//4
    static void init()
    {
        driveRightTalon = new WPI_TalonSRX(Constants.DRIVE_RIGHT_TALON_ID);//init controllers
        driveLeftTalon = new WPI_TalonSRX(Constants.DRIVE_LEFT_TALON_ID);
        driveRightVictor = new WPI_VictorSPX(Constants.DRIVE_RIGHT_VICTOR_ID);
        driveLeftVictor = new WPI_VictorSPX(Constants.DRIVE_LEFT_VICTOR_ID);
        
        driveRightVictor.follow(driveRightTalon);//setup followers so we cant set the motors within a gearbox to go opposite directions 
        driveLeftVictor.follow(driveLeftTalon);

        driveRightTalon.setInverted(true);//set one side inverted or else forward would spin the robot
        driveRightVictor.setInverted(true);

        driveRightTalon.setNeutralMode(NeutralMode.Brake);//easier to drive if on break imo
        driveLeftVictor.setNeutralMode(NeutralMode.Brake);
        driveRightTalon.setNeutralMode(NeutralMode.Brake);
        driveRightVictor.setNeutralMode(NeutralMode.Brake);

        driveRightTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);//config the feedback sensors, need it for auto
        driveLeftTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        
        driveLeftTalon.setSensorPhase(true);//makes sure the sensors positive corresponds to motor positive. need this or else PID wont work
        driveRightTalon.setSensorPhase(true);

        
        driveLeftTalon.config_kP(Constants.kPrimaryPIDSlot, Constants.drivekP);//setup some PID stuff
        driveLeftTalon.config_kI(Constants.kPrimaryPIDSlot, Constants.drivekI);
        driveLeftTalon.config_kD(Constants.kPrimaryPIDSlot, Constants.drivekD);
        
        driveRightTalon.config_kP(Constants.kPrimaryPIDSlot, Constants.drivekP);//and for the other side
        driveRightTalon.config_kI(Constants.kPrimaryPIDSlot, Constants.drivekI);
        driveRightTalon.config_kD(Constants.kPrimaryPIDSlot, Constants.drivekD);

        driveLeftTalon.configMotionProfileTrajectoryPeriod(Constants.pathPlannerTimeStepMs);//config the motion profile for auto to the time steps of PathPlanner
        driveRightTalon.configMotionProfileTrajectoryPeriod(Constants.pathPlannerTimeStepMs);
    }
    static void drive(Joystick stick)//single function to drive, called from telop
    {
        tank(stick.getRawAxis(1)+stick.getRawAxis(2), stick.getRawAxis(1)-stick.getRawAxis(2));
    }
    static void tank(double left, double right)//set motors on percent output specifically just in case the encoders failed
    {
        driveLeftTalon.set(ControlMode.PercentOutput, left);
        driveRightTalon.set(ControlMode.PercentOutput, right);
    }
}