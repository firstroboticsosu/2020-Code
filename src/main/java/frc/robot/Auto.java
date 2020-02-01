package frc.robot;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.control.AdaptivePurePursuitController;
import frc.lib.control.Path;
import frc.lib.control.Path.Waypoint;
import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Translation2d;
import frc.lib.geometry.Twist2d;
import frc.lib.util.DriveSignal;
import frc.robot.Drive.DriveControlState;

class Auto 
{
    // static BufferedTrajectoryPointStream pointStreaml;//the stream for the talons
    // static BufferedTrajectoryPointStream pointStreamr;
    // static TrajectoryPoint[][] autoBottom;//our two auto routines
    // static TrajectoryPoint[][] autoMiddle;
    // static void init()
    // {
        // System.out.println("Loading Motion Profiles");//Print we are loading so if we crash we know why
        // autoBottom = Pathing.loadByName("bottom");
        // autoMiddle = Pathing.loadByName("middle");
        // pointStreaml = new BufferedTrajectoryPointStream();
        // pointStreamr = new BufferedTrajectoryPointStream();
        // runningAutoMP = false;
        // Drive.driveLeftTalon.getSensorCollection().setQuadraturePosition(0, 10);//reset encoders because why not
        // Drive.driveRightTalon.getSensorCollection().setQuadraturePosition(0, 10);
    // }
    // static void autoStream(boolean bottom)//start the streams
    // {
    //     Drive.driveRightTalon.clearMotionProfileTrajectories();//creat out any old data in case we havent restarted the roborio since running auto last
    //     Drive.driveLeftTalon.clearMotionProfileTrajectories();
    //     if(bottom)//choose the correct auto to run and write it to the talons api
    //     {
    //         pointStreamr.Write(autoBottom[1]);
    //         pointStreaml.Write(autoBottom[0]);
    //     }
    //     else
    //     {
    //         pointStreamr.Write(autoMiddle[1]);
    //         pointStreaml.Write(autoMiddle[0]);
    //     }
    //     runningAutoMP = false;
    // }
    // static boolean runningAutoMP = false;//this makes sure we only start the stream onces
    // static void runAuto()
    // {
    //     if(!runningAutoMP)
    //     {
    //         Drive.driveRightTalon.startMotionProfile(pointStreamr, 10, ControlMode.MotionProfile);
    //         Drive.driveLeftTalon.startMotionProfile(pointStreaml, 10, ControlMode.MotionProfile);
    //         //start the motion profiles because we havent yet and write down we started them
    //         runningAutoMP = true;
    //     }
    //     if(Drive.driveRightTalon.isMotionProfileFinished())//if we are finished eject balls
    //     {
    //         Drive.tank(0, 0);//stop the motors
    //         Mech.doorPiston.set(Value.kReverse);
    //         if(Robot.autonTimer.get()<.5)//wait for the piston to go down and then run belt
    //         {
    //             Mech.harvySpark.set(0);
    //         }
    //         else
    //         {
    //             Mech.harvySpark.set(.3);
    //         }
    //     }
    //     else
    //     {
    //         Robot.autonTimer.reset();//keep timer at 0 till we done
    //         Mech.doorPiston.set(Value.kForward);//keep door closed and belt off until we are done with motion profile
    //         Mech.harvySpark.set(0);
    //     }
    //     System.out.println("Auto Status: "+Drive.driveRightTalon.isMotionProfileFinished()+"\t"+Drive.driveLeftTalon.getActiveTrajectoryPosition()+"\t"+Drive.driveRightTalon.getActiveTrajectoryPosition());
    // }

    
    static Path bottomPath;
    static Path middlePath;
    static void createPaths() 
    {
        ArrayList<Waypoint> middlePathList = new ArrayList<Waypoint>();
        middlePathList.add(new Path.Waypoint(new Translation2d(0, 0), 6));
        middlePathList.add(new Path.Waypoint(new Translation2d(0.3287074150956293,-4.598200271061528), 6));
        middlePathList.add(new Path.Waypoint(new Translation2d(5.647599301751222,-5.847333971715493), 6));
        middlePathList.add(new Path.Waypoint(new Translation2d(7.622036441494587,-5.242914439140994), 2));
        middlePathList.add(new Path.Waypoint(new Translation2d(8.427929151593919,-6.492048139794959), 2));
        middlePathList.add(new Path.Waypoint(new Translation2d(12.114888300298363,-6.089101784745292), 2));
        middlePathList.add(new Path.Waypoint(new Translation2d(15.640668906982944,-5.827186653963009), 2));
        middlePath = new Path(middlePathList);


        ArrayList<Waypoint> bottomPathList = new ArrayList<Waypoint>();
        bottomPathList.add(new Path.Waypoint(new Translation2d(0, 0), 6));
        bottomPathList.add(new Path.Waypoint(new Translation2d(0.5301805926204624, -6.7942579060822075), 6));
        bottomPathList.add(new Path.Waypoint(new Translation2d(5.647599301751222,-5.847333971715493), 6));
        bottomPathList.add(new Path.Waypoint(new Translation2d(7.622036441494587,-5.242914439140994), 2));
        bottomPathList.add(new Path.Waypoint(new Translation2d(8.427929151593919,-6.492048139794959), 2));
        bottomPathList.add(new Path.Waypoint(new Translation2d(12.114888300298363,-6.089101784745292), 2));
        bottomPathList.add(new Path.Waypoint(new Translation2d(15.640668906982944,-5.827186653963009), 2));
        bottomPath = new Path(bottomPathList);
    }
    static void reset()
    {
        pathFollowingController = null;
    }
    //copied from 4145 :)
    static AdaptivePurePursuitController pathFollowingController;
    
    public static void followPath(Path path, boolean reversed) {
        pathFollowingController = new AdaptivePurePursuitController(Constants.PATH_FOLLOWING_LOOKAHEAD,
                Constants.PATH_FOLLOWING_MAX_ACCELERATION, Constants.DRIVETRAIN_UPDATE_RATE, path, reversed, 1);
        Drive.driveControlState = DriveControlState.PATH_FOLLOWING_CONTROL;
        updatePathFollower();
    }
    static double metersToRotations(double meters) {
        return meters / (Constants.WHEEL_DIAMETER * Math.PI);
    }

    static double metersPerSecondToRpm(double mps) {
        return metersToRotations(mps) * 60;
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
        setVelocity(setpoint);
        // driveTank(metersPerSecondToRpm(setpoint.getLeft()), metersPerSecondToRpm(setpoint.getRight()));
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
        Drive.driveLeftTalon.set(ControlMode.Velocity, signal.getLeft());
        Drive.driveRightTalon.set(ControlMode.Velocity, signal.getRight());
    }
}