package frc.lib.drivers;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation;
//var<<8 shift var contents back 8
public class ColorSensor{

    private static final int DEFAULT_ADDR = 0x39;

    //register list
    private static final int ENABLE = 0x00;
    private static final int ATIME = 0x01;
    private static final int WTIME = 0x03;
    private static final int AILTL = 0x04;
    private static final int AILTH = 0x05;
    private static final int AIHTL = 0x06;
    private static final int AIHTH = 0x07;
    private static final int PILTL = 0x08;
    private static final int PILTH = 0x09;
    private static final int PIHTL = 0x0A;
    private static final int PIHTH = 0x0B;
    private static final int PERS = 0x0C;
    private static final int CONFIG = 0x0D;
    private static final int PPULSE = 0x0E;
    private static final int CONTROL = 0x0F;
    private static final int REVISION = 0x11;
    private static final int ID = 0x12;
    private static final int STATUS = 0x13;
    private static final int CDATAL = 0x14;
    private static final int CDATAH = 0x15;
    private static final int RDATAL = 0x16;
    private static final int RDATAH = 0x17;
    private static final int GDATAL = 0x18;
    private static final int GDATAH = 0x19;
    private static final int BDATAL = 0x1A;
    private static final int BDATAH = 0x1B;
    private static final int PDATAL = 0x1C;
    private static final int PDATAH = 0x1D;


    private I2C colorI2c;
    private byte[] read_data;
    private int address;
    
    public ColorSensor(int address){
        this.address = address;
        reset();

    }

    public ColorSensor(){
        this(0x39);
    }

    /**
     * reinitilaizes the color sensor on the I2C address specified
     */
    public void reset(){
        if(colorI2c != null) colorI2c.close();
        colorI2c = new I2C(I2C.Port.kOnboard, address);
        if(colorI2c.addressOnly()) DriverStation.reportError("Could not init color sensor at address " + address, false);
        else{
            colorI2c.write(ENABLE, 0b00010011); // write Abient color sensing on, adc enable and power on for timer enable
            colorI2c.write(CONFIG, 0x00); //configure WLONG off to read fast
        }
    }

    /**
     * gets the latest color reading from the sensor
     * @return an aray of doubles between 0 and 1 orderd R,G,B
     */
    public double[] getColor(){
        double[] out = new double[3];
        colorI2c.read(RDATAL, 2, read_data);
        if(read_data.length > 1){
            out[0] = (read_data[1] << 8 | read_data[0]) / (double)0xffff;
        }
        colorI2c.read(GDATAL, 2, read_data);
        if(read_data.length > 1){
            out[1] = (read_data[1] << 8 | read_data[0]) / (double)0xffff;
        }
        colorI2c.read(GDATAL, 2, read_data);
        if(read_data.length > 1){
            out[2] = (read_data[1] << 8 | read_data[0]) / (double)0xffff;
        }
        return out;
    }

}