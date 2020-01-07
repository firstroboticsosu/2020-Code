package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private DoubleSolenoid collectorPiston;

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
        collectorPiston.set(periodicIO.collector_down ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward);
    }

    private Ramp() {
        lowerRampMotor = new CANSparkMax(Constants.RAMP_ID, MotorType.kBrushless);
        flapPiston = new DoubleSolenoid(Constants.FLAP_PISTON_FORWARD_ID, Constants.FLAP_PISTON_REVERSE_ID);
        collectorPiston = new DoubleSolenoid(Constants.COLLECTOR_FORWARD_ID, Constants.COLLECTOR_REVERSE_ID);
        configTalons();
        reset();
    }

    public void reset() {
        periodicIO = new Ramp.RampIO();
        flapPiston.set(DoubleSolenoid.Value.kOff);
        collectorPiston.set(DoubleSolenoid.Value.kOff);
    }

    private void configTalons() {
        // TODO
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
        public boolean collector_down = false;
    }

    public void spin(boolean forwards) {
        periodicIO.lower_ramp_demand = forwards ? Constants.LOWER_RAMP_UP_SPEED : Constants.LOWER_RAMP_DOWN_SPEED;
    }
    public void stopSpinning() {
        periodicIO.lower_ramp_demand = 0;
    }

    public void setRoller(boolean wantDown) {
        periodicIO.collector_down = wantDown;
    }

    public void setDoor(boolean wantClosed) {
        periodicIO.flap_closed = wantClosed;
    }

}
