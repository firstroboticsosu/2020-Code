package frc.robot;

import java.io.IOException;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

class Auto 
{
    static BufferedTrajectoryPointStream pointStreaml;//the stream for the talons
    static BufferedTrajectoryPointStream pointStreamr;
    static TrajectoryPoint[][] autoBottom;//our two auto routines
    static TrajectoryPoint[][] autoMiddle;
    static void init() throws IOException
    {
        System.out.println("Loading Motion Profiles");//Print we are loading so if we crash we know why
        autoBottom = Pathing.loadByName("bottom");
        autoMiddle = Pathing.loadByName("middle");
        pointStreaml = new BufferedTrajectoryPointStream();
        pointStreamr = new BufferedTrajectoryPointStream();
        runningAutoMP = false;
        Drive.driveLeftTalon.getSensorCollection().setQuadraturePosition(0, 10);//reset encoders because why not
        Drive.driveRightTalon.getSensorCollection().setQuadraturePosition(0, 10);
    }
    static void autoStream(boolean bottom)//start the streams
    {
        Drive.driveRightTalon.clearMotionProfileTrajectories();//creat out any old data in case we havent restarted the roborio since running auto last
        Drive.driveLeftTalon.clearMotionProfileTrajectories();
        if(bottom)//choose the correct auto to run and write it to the talons api
        {
            pointStreamr.Write(autoBottom[1]);
            pointStreaml.Write(autoBottom[0]);
        }
        else
        {
            pointStreamr.Write(autoMiddle[1]);
            pointStreaml.Write(autoMiddle[0]);
        }
        runningAutoMP = false;
    }
    static boolean runningAutoMP = false;//this makes sure we only start the stream onces
    static void runAuto()
    {
        if(!runningAutoMP)
        {
            Drive.driveRightTalon.startMotionProfile(pointStreamr, 10, ControlMode.MotionProfile);
            Drive.driveLeftTalon.startMotionProfile(pointStreaml, 10, ControlMode.MotionProfile);
            //start the motion profiles because we havent yet and write down we started them
            runningAutoMP = true;
        }
        if(Drive.driveRightTalon.isMotionProfileFinished())//if we are finished eject balls
        {
            Drive.tank(0, 0);//stop the motors
            Mech.doorPiston.set(Value.kReverse);
            if(Robot.autonTimer.get()<.5)//wait for the piston to go down and then run belt
            {
                Mech.harvySpark.set(0);
            }
            else
            {
                Mech.harvySpark.set(.3);
            }
        }
        else
        {
            Robot.autonTimer.reset();//keep timer at 0 till we done
            Mech.doorPiston.set(Value.kForward);//keep door closed and belt off until we are done with motion profile
            Mech.harvySpark.set(0);
        }
        System.out.println("Auto Status: "+Drive.driveRightTalon.isMotionProfileFinished()+"\t"+Drive.driveLeftTalon.getActiveTrajectoryPosition()+"\t"+Drive.driveRightTalon.getActiveTrajectoryPosition());
    }
}