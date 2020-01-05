package frc.lib.drivers;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalOutput;
//var<<8 shift var contents back 8
public class Color_sensor{

    private DigitalOutput outputPin;
    private Counter inputPin;
    private double distance;
    private static I2C colorI2c;
    static public int mode = 0;

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
    private static final int CDATA = 0x14;
    private static final int CDATAH = 0x15;
    private static final int RDATA = 0x16;
    private static final int RDATAH = 0x17;
    private static final int GDATA = 0x18;
    private static final int GDATAH = 0x19;
    private static final int BDATA = 0x1A;
    private static final int BDATAH = 0x1B;
    private static final int PDATA = 0x1C;
    private static final int PDATAH = 0x1D;


    private byte[] read_data;
    private int rdata;
    private int bdata;
    private int gdata;
    //private int[] address_book={};

    public Color_sensor(int inputPin, int outputPin){
        this.outputPin = new DigitalOutput(outputPin);
        this.inputPin = new Counter(inputPin);
        this.inputPin.setSemiPeriodMode(true);
    }

    public void reader(int registerAddress)
    {

            colorI2c.read(registerAddress, 8, read_data);
    }

    public void writer(int registerAddress, int data)
    {

            colorI2c.write(registerAddress, data);
    }
    public void update(int i){
        outputPin.set(true);
        distance = inputPin.getPeriod()*1000000/148;
        outputPin.set(false);
    }

    public double getColor(){
        update();
        return distance;
    }

}