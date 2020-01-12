package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Spark;
import frc.robot.Constants;

 
public class Climb extends Subsystem{

    private TalonSRX roller;
    private Spark winch;
    private ClimbIO periodicIO;

    private static final Climb instance = new Climb();

    public static Climb getInstance(){
        return instance;
    }

    private Climb(){
        roller = new TalonSRX(Constants.LIFT_ROLLER_ID);
        winch = new Spark(Constants.LIFT_WINCH_ID);
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
	public void outputTelemetry() {		}

	@Override
	public void reset() {
		periodicIO = new ClimbIO();
    }
    
    private void configMotors(){
        //config Talon
        roller.setInverted(false);
        roller.setNeutralMode(NeutralMode.Brake);

    }

	@Override
	public void onStop() {	}

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