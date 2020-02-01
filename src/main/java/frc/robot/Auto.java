package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.lib.control.Path;
import frc.lib.control.Path.Waypoint;
import frc.lib.geometry.Translation2d;
import frc.lib.util.DriveSignal;

class Auto 
{    
    static Path bottomPath;
    static Path middlePath;
    static String m_autoSelected = "";

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
    
    public static void autoInit(String autoSelected){
        m_autoSelected = autoSelected;// Constants.kBottom
        System.out.println("Auto selected: " + m_autoSelected);
        switch (m_autoSelected) {
            case Constants.kBottom:
                Drive.followPath(Auto.bottomPath, false);
            break;

            case Constants.kMiddle:
                Drive.followPath(Auto.middlePath, false);
            break;

                case Constants.kDefaultAuto:

                break;

            default:
            break;
        }

    }

    public static void autoUpdate(double autoTime){

        switch (m_autoSelected) {
            case Constants.kBottom:
                Drive.updatePathFollower();
            break;

            case Constants.kMiddle:
                Drive.updatePathFollower();
            break;

            default:
                if (autoTime < 2.0) {
                    //Drive.setVelocity(new DriveSignal(Drive.RPMToUnitsPer100Ms(Drive.metersPerSecondToRpm(1)),
                        //Drive.RPMToUnitsPer100Ms(Drive.metersPerSecondToRpm(1))));
                    Drive.tank(0.5, 0.5);
                    Mech.doorPiston.set(Value.kForward);
                    Mech.harvySpark.set(0);
                } else {
                    Drive.tank(0, 0);
                    //Mech.doorPiston.set(Value.kReverse);
                    //Mech.harvySpark.set(.3);
                }
            break;
        }
    }

}