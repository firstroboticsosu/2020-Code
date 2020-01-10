package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.robot.Constants;

public class Ramp extends Subsystem {

    //construct one and only 1 instance of this class
    private static Ramp m_RampInstance = new Ramp();

    public static Ramp getInstance() {
        return m_RampInstance;
    }

    //used internally for data
    private RampIO periodicIO;

    // Hardware
    private CANSparkMax lowerRampMotor;
    private DoubleSolenoid flapPiston;

    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Ramp.this) {

                }
            }

            @Override
            public void onStop(double timestamp) {

            }
        });
    }

    @Override
    public synchronized void readPeriodicInputs() {
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        lowerRampMotor.set(periodicIO.lower_ramp_demand);
        flapPiston.set(periodicIO.flap_closed ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward);
    }

    private Ramp() {
        lowerRampMotor = new CANSparkMax(Constants.RAMP_ID, MotorType.kBrushless);
        flapPiston = new DoubleSolenoid(Constants.FLAP_PISTON_FORWARD_ID, Constants.FLAP_PISTON_REVERSE_ID);
        configSparkMax();
        reset();
    }


    public void reset() {
        periodicIO = new Ramp.RampIO();
        flapPiston.set(DoubleSolenoid.Value.kOff);
    }

    private void configSparkMax() {
        //TODO figure out how to clear errors with spark max on CANbus
    }

    @Override
    public PeriodicIO getLogger() {
        return periodicIO;
    }

    public void onStop(){

    }

    public void outputTelemetry() {
    }

    public class RampIO extends PeriodicIO {
        // INPUTS
        // Operator input

        // OUTPUTS
        // Set desired output values
        public double lower_ramp_demand = 0.0;
        public boolean flap_closed = true;
    }

    public void spin(double power) {
        periodicIO.lower_ramp_demand = power;
    }
    public void stopSpinning() {
        periodicIO.lower_ramp_demand = 0;
    }
    
    public void setDoor(boolean wantClosed) {
        periodicIO.flap_closed = wantClosed;
    }

}
