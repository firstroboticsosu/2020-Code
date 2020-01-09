package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;

 
public class Climb extends Subsystem{

    private TalonSRX roller;
    private CANSparkMax winch;
    private ClimbIO periodicIO;

    private static final Climb instance = new Climb();

    public static Climb getInstance(){
        return instance;
    }

    private Climb(){
        roller = new TalonSRX(Constants.LIFT_ROLLER_ID);
        winch = new CANSparkMax(Constants.LIFT_WINCH_ID, MotorType.kBrushless);   
        reset();
    }

	@Override
	public void readPeriodicInputs() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void writePeriodicOutputs() {
        winch.set(periodicIO.winchDemand);
        roller.set(ControlMode.PercentOutput, periodicIO.rollerDemand);
		
	}

	@Override
	public void outputTelemetry() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void reset() {
		periodicIO = new ClimbIO();
		
	}

	@Override
	public void onStop() {
		// TODO Auto-generated method stub
		
    }

    public void setRoller(double power){
        periodicIO.rollerDemand = power;
    }

    public void setWinch(double power){
        periodicIO.winchDemand = power;
    }
    
    public class ClimbIO extends PeriodicIO{
        public double winchDemand = 0.0;
        public double rollerDemand = 0.0;
    }



}