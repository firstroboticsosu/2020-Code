package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C.Port;

public class Lighting 
{
    static final byte DATA_MODE = 0;
    private static Lighting lights = new Lighting();
    private static I2C lightsI2c;
    static long frame = 0;
    byte currentMode = 0;
    static public int mode = 0;
    Alliance knownAlliance = Alliance.Invalid;
    static public Lighting getInstance()
    {
        return lights;
    }
    static public void reset() {
        if(lightsI2c!=null) lightsI2c.close();
        lightsI2c = new I2C(Port.kOnboard, 9);
    }
    public void writeMode(int i)
    {
        if(mode!=i)
        {
            lightsI2c.writeBulk(new byte[]{0,(byte)i});
            mode = i;
        }
    }
    public void writeAlliance(boolean red)
    {
        lightsI2c.writeBulk(new byte[]{1, (byte) (red ? 1 : 0) });
    }
    public void writeBallCount(int c)
    {
        lightsI2c.writeBulk(new byte[]{1, (byte) (c) });
    }
    public void off()
    {
        writeMode(0);
    }
    public void enabled()
    {
        writeMode(2);
    }
    public void disabled()
    {
        writeMode(1);
    }
    public void auto()
    {
        writeMode(3);
    }
    public void disBallCount()
    {
        writeMode(4);
    }
    public void setBallCount(int c)
    {
        writeBallCount(c)
    }
    public void gotAlliance(boolean red)
    {
        writeAlliance(red);
    }
    public void determineAlliance(Alliance alliance) 
    {
        if(alliance!=knownAlliance)
        {
            writeAlliance(alliance==Alliance.Red);
            knownAlliance = alliance;
        }
	}
}