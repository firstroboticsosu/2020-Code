package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C.Port;
import frc.robot.Sensors.MyColor;

public class Lighting//my favorite class, controls the LEDs to match robot state
{
    private static I2C lightsI2c;//the communication object for i2c from WPI
    static public int mode = 0;//current lights mode, keep track so that we only have to send it once
    static Alliance knownAlliance = Alliance.Invalid;//the alliance, keep track so we only have to send once
    static boolean wasAttached = false;//was attached to driver station, keep track so we know when we lose connection
    static boolean wasTele = false;//was telop, keep track so we know when we were in telops
    static MyColor currentDisplayed = MyColor.Unknown;//currently displayed color, Unknown = nothing displayed, keep track so we only have to send once
    static public void reset() {//reset the i2c port, used for init
        if(lightsI2c!=null) lightsI2c.close();
        lightsI2c = new I2C(Port.kOnboard, 9);
    }
    static void writeMode(int i)//write the current mode if not in that mode rn
    {
        if(mode!=i)
        {
            lightsI2c.writeBulk(new byte[]{0,(byte)i});
            mode = i;
        }
    }
    static void writeAlliance(byte by)//write out alliance
    {
        lightsI2c.writeBulk(new byte[]{1, by});
    }
    static void writeBallCount(int c)//was going to display balls we are holding but never got the sensor to do it
    {
        lightsI2c.writeBulk(new byte[]{2, (byte) (c) });
    }
    static void off()//turns the lights off (but actually plays fire, dont tell anyone but you cant acutually turn them off)
    {
        writeMode(0);
    }
    static void telop()//write we are in telop
    {
        writeMode(2);
    }
    static void disabled()//write we are disabled
    {
        writeMode(1);
    }
    static void auto()//write we are in auto
    {
        writeMode(3);
    }
    static void disBallCount()//not used
    {
        writeMode(4);
    }
    static void matchEnded()//write match ended
    {
        writeMode(5);
    }
    static void setBallCount(int c)//not used
    {
        writeBallCount(c);
    }
    static void determineAlliance(Alliance alliance)//determins if we changed alliance and displays it
    {
        if(alliance!=knownAlliance)
        {
            if(alliance==Alliance.Red)
            {
                writeAlliance((byte)0);
            }
            if(alliance==Alliance.Blue)
            {
                writeAlliance((byte)1);
            }
            if(alliance==Alliance.Invalid)
            {
                writeAlliance((byte)2);
            }
            knownAlliance = alliance;
        }
    }
    static void disconnected()//when we disconnect tell the lights we no longer know our alliance
    {
        writeAlliance((byte)2);
    }
    public static void ds_attached(boolean dsAttached) //tell when we are unplugeds
    {
        if(!dsAttached && wasAttached)
        {
            disconnected();
        }
        wasAttached = dsAttached;
    }
    static void checkMatchEnded(DriverStation m_ds)//check if we were in telop and now are not. Used to determine if we just played a match and display something different
    {
        if(m_ds.isOperatorControl() && !m_ds.isDisabled())
        {
            wasTele = true;
        }
        if(wasTele && m_ds.isDisabled())
        {
            wasTele = false;
            matchEnded();
        }
    }
    public static void setColor(MyColor color) //set the color to something we are seeing with the color sensor
    {
        if(color!=currentDisplayed)
        {
            if(DriverStation.getInstance().isOperatorControl())
            {
                color = MyColor.Unknown;
            }
            sendColor(color);
        }
	}
    static void sendColor(MyColor color) //actually send it and write down we did
    {
        System.out.println("Displayed: "+color+"\t"+colorToByte(color));
        currentDisplayed = color;
        lightsI2c.writeBulk(new byte[]{3, (byte) (colorToByte(color))});
    }
    static int colorToByte(MyColor color)//change the color enum to something sendable over i2c
    {
        for(int i = 0; i < MyColor.values().length; i++)
        {
            if(color==MyColor.values()[i])
            {
                return i;
            }
        }
        return -1;
    }
}