package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMax;
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
    private IntakeControlState mRampControlState = IntakeControlState.INACTIVE;
    private OutputControlState mOutputControlState = OutputControlState.INACTIVE;
    private CollectorControlState mCollectorControlState = CollectorControlState.INACTIVE;
    private RampIO periodicIO;

    // Hardware
    private TalonSRX upperRampMotor;
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
        //TODO read in button info
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        upperRampMotor.set(ControlMode.PercentOutput, periodicIO.upper_ramp_demand);
        lowerRampMotor.set(periodicIO.lower_ramp_demand);
        flapPiston.set(periodicIO.flap_closed ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward);
        collectorPiston.set(periodicIO.collector_down ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward);
    }

    private Ramp() {
        lowerRampMotor = new CANSparkMax(Constants.COLLECTOR_ID, MotorType.kBrushless);
        upperRampMotor = new TalonSRX(Constants.RAMP_ID);
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
        ErrorCode sensorPresent;
        sensorPresent = upperRampMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100); //primary closed-loop, 100 ms timeout
        if (sensorPresent != ErrorCode.OK) {
            DriverStation.reportError("Could not detect left encoder: " + sensorPresent, false);
        }
        upperRampMotor.setSensorPhase(true);
        upperRampMotor.selectProfileSlot(0, 0);
        upperRampMotor.config_kF(0, Constants.UPPER_RAMP_KF, 0);
        upperRampMotor.config_kP(0, Constants.UPPER_RAMP_KP, 0);
        upperRampMotor.config_kI(0, Constants.UPPER_RAMP_KI, 0);
        upperRampMotor.config_kD(0, Constants.UPPER_RAMP_KD, 0);
        upperRampMotor.config_IntegralZone(0, 300);
        upperRampMotor.setInverted(false);
        upperRampMotor.setNeutralMode(NeutralMode.Brake);
        upperRampMotor.configVoltageCompSaturation(Constants.RAMP_VCOMP);
        upperRampMotor.enableVoltageCompensation(true);
    }

    @Override
    public PeriodicIO getLogger() {
        return periodicIO;
    }

    public void onStop(){

    }

    enum IntakeControlState {
        INACTIVE,
        UP,
        DOWN;

        public String toString() {
            return name().charAt(0) + name().substring(1).toLowerCase();
        }
    }

    enum OutputControlState {
        INACTIVE,
        UP,
        DOWN;

        public String toString() {
            return name().charAt(0) + name().substring(1).toLowerCase();
        }
    }

    enum CollectorControlState {
        INACTIVE,
        UP,
        DOWN;

        public String toString() {
            return name().charAt(0) + name().substring(1).toLowerCase();
        }
    }

    public void outputTelemetry() {
        SmartDashboard.putString("Drive/Drive State", mRampControlState.toString());
        SmartDashboard.putString("Drive/Drive State", mOutputControlState.toString());
    }

    public class RampIO extends PeriodicIO {
        // INPUTS
        // Operator input
        public boolean forwardButtonPressed = false;
        public boolean backwardButtonPressed = false;

        public boolean flapUpButtonPressed = false;
        public boolean flapDownButtonPressed = false;

        // OUTPUTS
        // Set desired output values
        public double lower_ramp_demand = 0.0;
        public double upper_ramp_demand = 0.0;
        public boolean flap_closed = true;
        public boolean collector_down = false;
    }

    public void spinUp() {
        periodicIO.lower_ramp_demand = Constants.LOWER_RAMP_UP_SPEED;
        periodicIO.upper_ramp_demand = Constants.UPPER_RAMP_UP_SPEED;
    }
    public void spinDown() {
        periodicIO.lower_ramp_demand = Constants.LOWER_RAMP_DOWN_SPEED;
        periodicIO.upper_ramp_demand = Constants.UPPER_RAMP_DOWN_SPEED;
    }
    public void stopSpinning() {
        periodicIO.lower_ramp_demand = 0;
        periodicIO.upper_ramp_demand = 0;
    }

    public void setCollector(boolean wantDown) {
        periodicIO.collector_down = wantDown;
    }

    public void setDoor(boolean wantClosed) {
        periodicIO.flap_closed = wantClosed;
    }

}
