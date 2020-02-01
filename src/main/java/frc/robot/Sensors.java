package frc.robot;


import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C.Port;
import frc.lib.geometry.Rotation2d;


public class Sensors {//does the communication with the color sensor. it is mostly all done with the api
    
    static ColorSensorV3 cSensor;//red, yellow,green,cyan
    static int[][] colors = new int[][]{{1800,1300,550},{2500,4500,1100},{650,2200,1000},{200,2400,2500}};
    static int red = 0;
    static int green = 0;
    static int blue = 0;
    static int prox = 0;

    static Rotation2d gyro_heading;
    static Rotation2d gyro_offset;
    static PigeonIMU pidgey;

    enum MyColor
    {
        Red, Yellow, Green, Cyan, Unknown
    }
    static public void init() {
        cSensor = new ColorSensorV3(Port.kOnboard);
        pidgey = new PigeonIMU(Constants.PIDGEY_ID);
    }
    
    public static Rotation2d getHeading() {
        return gyro_heading;
    }

    public static void setHeading(Rotation2d heading) {
        System.out.println("SET HEADING: " + heading.getDegrees());
        gyro_offset = heading.rotateBy(Rotation2d.fromDegrees(pidgey.getFusedHeading()).inverse());
        System.out.println("Gyro offset: " + gyro_offset.getDegrees());
        gyro_heading = heading;
    }
    static void pollSensors()
    {
        red = cSensor.getRed();
        green = cSensor.getGreen();
        blue = cSensor.getBlue();
        prox = cSensor.getProximity();
        gyro_heading = Rotation2d.fromDegrees((pidgey.getFusedHeading()+360)%360);
    }
    static MyColor getFieldColor()//get what color the field system sees, assuming your controlling from your side of the alliance
    {
        MyColor c = getColor();
        switch(c)
        {
            case Red:
                return MyColor.Cyan;
            case Yellow:
                return MyColor.Green;
            case Cyan:
                return MyColor.Red;
            case Green:
                return MyColor.Yellow;
            default :
                return MyColor.Unknown;
        }
    }
    static MyColor getColor()//determinds the nearest color to what is being sensed, maybe need to be recalibrated in field conditions
    {
        if(prox<300)
        {
            return MyColor.Unknown;
        }
        else
        {
            int score = Integer.MAX_VALUE;
            int indexOfBest = -1;
            for(int i = 0; i < colors.length; i++)
            {
                int cScore = Math.abs(red-colors[i][0])+Math.abs(green-colors[i][1])+Math.abs(blue-colors[i][2]);
                if(cScore<score)
                {
                    score = cScore;
                    indexOfBest = i;
                }
            }
            return MyColor.values()[indexOfBest];
        }
    }
}