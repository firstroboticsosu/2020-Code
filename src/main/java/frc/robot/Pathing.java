package frc.robot;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;

import com.ctre.phoenix.motion.TrajectoryPoint;

public class Pathing {//this class does all the processing of the path data we saved on the roborio
    static String source = "/home/lvuser/deploy/";//this is where the "deploy" folder goes to on the actual linux system
    static String leftFileName;
    static String rightFileName;
    
    static TrajectoryPoint[][] loadByName(String name) throws IOException //load a path by its name 
    {
        leftFileName = source + name +"_left.csv";//we have one file for each side so figure that out
        rightFileName = source + name + "_right.csv";
        BufferedReader readerl = new BufferedReader(new FileReader(leftFileName));//open the file readers
        BufferedReader readerr = new BufferedReader(new FileReader(rightFileName));
        String line = "";
        String alll = "";
        String allr = "";
        while (line != null) {//while there is more to read, contine
            line = readerl.readLine();
            if(line!=null)
            {
                alll += line + "\n";
            }
            line = readerr.readLine();
            if(line!=null)
            {
                allr += line + "\n";
            }
        }
        readerl.close();//close the readers
        readerr.close();
        String[] linesl = alll.split("\n");
        String[] linesr = allr.split("\n");
        TrajectoryPoint[][] points = new TrajectoryPoint[2][linesl.length];//create something to return back
        for(int i = 0; i < points[0].length; i++)
        {
            TrajectoryPoint pointl = new TrajectoryPoint();
            TrajectoryPoint pointr = new TrajectoryPoint();


            String[] singlel = linesl[i].split(",");//get all the values and store them
            double positionl = Double.parseDouble(singlel[0]);
            double vell = Double.parseDouble(singlel[1]);
            // double accell = Double.parseDouble(singlel[2]);
            pointl.timeDur = 0;//most of this was copied from the example and adapted to meet our requrements
            pointl.position = positionl * Constants.kSensorUnitsPerMeter;           // Convert Revolutions to Units
            pointl.velocity = vell * Constants.kSensorUnitsPerMeter / 10.0;   // Convert m/s to Units/100ms
            pointl.auxiliaryPos = 0;
            pointl.auxiliaryVel = 0;
            pointl.profileSlotSelect0 = Constants.kPrimaryPIDSlot; /* which set of gains would you like to use [0,3]? */
            pointl.profileSlotSelect1 = 0; /* auxiliary PID [0,1], leave zero */
            pointl.zeroPos = (i == 0); /* set this to true on the first point */
            pointl.isLastPoint = ((i + 1) == linesl.length); /* set this to true on the last point */
            pointl.arbFeedFwd = 0; /* you can add a constant offset to add to PID[0] output here */

            String[] singler = linesr[i].split(",");//same thing for the right side
            double positionr = Double.parseDouble(singler[0]);
            double velr = Double.parseDouble(singler[1]);
            // double acceleration = Double.parseDouble(singler[2]);
            pointr.timeDur = 0;
            pointr.position = positionr * Constants.kSensorUnitsPerMeter;
            pointr.velocity = velr * Constants.kSensorUnitsPerMeter / 10.0;
            pointr.auxiliaryPos = 0;
            pointr.auxiliaryVel = 0;
            pointr.profileSlotSelect0 = Constants.kPrimaryPIDSlot; /* which set of gains would you like to use [0,3]? */
            pointr.profileSlotSelect1 = 0; /* auxiliary PID [0,1], leave zero */
            pointr.zeroPos = (i == 0); /* set this to true on the first point */
            pointr.isLastPoint = ((i + 1) == linesl.length); /* set this to true on the last point */
            pointr.arbFeedFwd = 0; /* you can add a constant offset to add to PID[0] output here */
            points[0][i] = pointl;//save them to the array to return
            points[1][i] = pointr;
        }
        System.out.println("Loaded "+points[0].length+" Path Points");//nice print out
        return points;
    }
}