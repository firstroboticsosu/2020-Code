package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Sensors.MyColor;

public class Mech//hold  the motors and relevant code for the contorl of those motors 
{
    static WPI_TalonSRX spinnderTalon;//5, all the objects holding the various mech actuators
    static WPI_TalonSRX climbRollerTalon;//6
    static CANSparkMax harvySpark;
    static CANSparkMax climberSpark;
    static DoubleSolenoid doorPiston;
    static DoubleSolenoid colorSensorPiston;
    static Compressor compressor;
    static Timer spinUpTimer;
    static Timer spinDownTimer;
    static int encOffsetFor3spin = 0;//tells us if we have reset 
    static void init()
    {
        spinnderTalon = new WPI_TalonSRX(Constants.MECH_SPINNER_TALON_ID);
        climbRollerTalon = new WPI_TalonSRX(Constants.MECH_CLIMBER_TALON_ID);
        harvySpark = new CANSparkMax(Constants.MECH_HARVY_SPARK_ID, MotorType.kBrushless);
        climberSpark = new CANSparkMax(Constants.MECH_HARVY_CLIMBER_ID, MotorType.kBrushless);
        compressor = new Compressor();
        doorPiston = new DoubleSolenoid(0, 1);
        colorSensorPiston = new DoubleSolenoid(4, 5);
        spinUpTimer = new Timer();
        spinDownTimer = new Timer();

        spinnderTalon.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition, 0, 10);
        spinnderTalon.setNeutralMode(NeutralMode.Brake);
        spinnderTalon.setSensorPhase(true);

        climberSpark.setIdleMode(IdleMode.kBrake);
        
        resetActuators();
    }
    static void mech(Joystick stick)
    {
        setSpinner(stick);
        /*if(stick.getRawButton(1))
        {
            harvySpark.set(.3);
        }
        else if(stick.getRawButton(2))
        {
            harvySpark.set(-.3);
        }
        else
        {
            harvySpark.set(0);
        }*/
        if(stick.getRawButton(2))
        {
            harvySpark.set(stick.getRawAxis(1)/2);
        }
        else
        {
            harvySpark.set(stick.getRawAxis(1)/5);
        }
        if(stick.getRawButton(4))
        {
            climberSpark.set(.5);
        }
        else
        {
            climberSpark.set(0);
        }
        if(stick.getPOV()==90)
        {
            climbRollerTalon.set(.5);
        }
        else if(stick.getPOV()==270)
        {
            climbRollerTalon.set(-.5);
        }
        else
        {
            climbRollerTalon.set(0);
        }
        if(stick.getRawButton(5))
        {
            doorPiston.set(Value.kForward);
        }
        if(stick.getRawButton(6))
        {
            doorPiston.set(Value.kReverse);
        }
    }
    static void setSpinner(Joystick stick)
    {
        double stickVal = stick.getRawAxis(3)-stick.getRawAxis(4);
        if(Math.abs(stickVal)>.05)
        {
            controlSensorPiston(stickVal);
        }
        else
        {            
            if(stick.getRawAxis(5)<-.3)//spin 3 times
            {
                if(encOffsetFor3spin<0)
                {
                    encOffsetFor3spin = spinnderTalon.getSelectedSensorPosition();
                    spinnderTalon.setSelectedSensorPosition(0);
                }
                spinnderTalon.set(ControlMode.Position, Constants.spin3ticks, DemandType.ArbitraryFeedForward, 0);
            }
            else
            {
                if(stick.getRawAxis(5)>.3)//spin 3 times
                {
                    runUntilFindColor(DriverStation.getInstance().getGameSpecificMessage());
                }
                else
                {
                    controlSensorPiston(0);
                }
                encOffsetFor3spin = -1;
            }
        }
    }
    public static void controlSensorPiston(double val)
    {
        if(Math.abs(val)>.05)//put piston out when you want to move
        {
            if(spinUpTimer.get()==0)
            {
                spinUpTimer.start();
                colorSensorPiston.set(Value.kForward);
                spinnderTalon.set(0);
            }
            else if(spinUpTimer.get()>.1)
            {
                colorSensorPiston.set(Value.kForward);
                spinnderTalon.set(val);
            }
        }
        else
        {
            spinnderTalon.set(0);
            if(spinUpTimer.get()>0)
            {
                spinUpTimer.stop();
                spinUpTimer.reset();
                spinDownTimer.reset();
                spinDownTimer.start();
            }
            if(spinDownTimer.get()!=0 && spinDownTimer.get() < 1)
            {
                colorSensorPiston.set(Value.kForward);
            }
            else
            {
                spinDownTimer.stop();
                colorSensorPiston.set(Value.kReverse);
            }
        }
    }
    public static void resetActuators() 
    {
        spinnderTalon.set(0);
        harvySpark.set(0);
        climberSpark.set(0);
        climbRollerTalon.set(0);
        doorPiston.set(Value.kForward);
        colorSensorPiston.set(Value.kReverse);
    }
    public static void runUntilFindColor(String gameData)
    {
        System.out.println(gameData);
        boolean good = isGood(gameData);
        if(!good)
        {
            controlSensorPiston(1);
        }
        else
        {
            spinnderTalon.set(0);
        }
    }
    public static boolean isGood(String gameData)
    {
        MyColor color = Sensors.getColor();
        boolean good = false;
        if(gameData.length() > 0)
        {
            switch (gameData.charAt(0))
            {
                case 'B' ://run until red
                    good = color==MyColor.Red;
                break;
                case 'G' ://run until yellow
                    good = color==MyColor.Yellow;
                break;
                case 'R' ://run until cyan
                    good = color==MyColor.Cyan;
                break;
                case 'Y' ://run until green
                    good = color==MyColor.Green;
                break;
                default :
                System.out.println("BAD GAME DATA");
                break;
            }
        }
        System.out.println(good ? "COLOR GOOD" : "COLOR BAD");
        return good;
    }
}