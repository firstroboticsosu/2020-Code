package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
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
    private TalonSRX rampMotor;
    private Piston flapPiston;
    private Piston collectorPiston;

    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Ramp.this) {

                    switch (mRampControlState) {

                        case INACTIVE:
                            periodicIO.ramp_demand = 0;
                            break;

                        case UP:
                            periodicIO.ramp_demand = Constants.RAMP_UP_SPEED;

                        case DOWN:
                            periodicIO.ramp_demand = Constants.RAMP_DOWN_SPEED;
                            break;

                        default:
                            System.out.println("Unexpected control state for input motor");
                    }

                    switch (mOutputControlState) {

                        case INACTIVE:
                            break;

                        case UP:
                            periodicIO.flap_piston_distance = Constants.FLAP_PISTON_UP_VALUE;
                            break;

                        case DOWN:
                            periodicIO.flap_piston_distance = Constants.FLAP_PISTON_DOWN_VALUE;
                            break;

                        default:
                            System.out.println("Unexpected control state for plate controller");

                    }

                    switch (mCollectorControlState) {

                        case INACTIVE:
                            periodicIO.ramp_demand = 0;
                            break;

                        case UP:
                            periodicIO.collector_piston_distance = Constants.COLLECTOR_PISTON_UP_VALUE;

                        case DOWN:
                            periodicIO.collector_piston_distance = Constants.COLLECTOR_PISTON_DOWN_VALUE;
                            break;

                        default:
                            System.out.println("Unexpected control state for input motor");
                    }

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
        rampMotor.set(ControlMode.PercentOutput, periodicIO.ramp_demand);
        // TODO Add piston to this
    }

    private Ramp() {
        rampMotor = new TalonSRX(Constants.RAMP_ID);
        flapPiston = new ; // TODO
        collectorPiston = new ; // TODO
        configTalons();
        reset();
    }

    public void reset() {
        periodicIO = new Ramp.RampIO();

    }

    private void configTalons() {
        ErrorCode sensorPresent;
        sensorPresent = rampMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100); //primary closed-loop, 100 ms timeout
        if (sensorPresent != ErrorCode.OK) {
            DriverStation.reportError("Could not detect left encoder: " + sensorPresent, false);
        }
        rampMotor.setSensorPhase(true);
        rampMotor.selectProfileSlot(0, 0);
        rampMotor.config_kF(0, Constants.RAMP_KF, 0);
        rampMotor.config_kP(0, Constants.RAMP_KP, 0);
        rampMotor.config_kI(0, Constants.RAMP_KI, 0);
        rampMotor.config_kD(0, Constants.RAMP_KD, 0);
        rampMotor.config_IntegralZone(0, 300);
        rampMotor.setInverted(false);
        rampMotor.setNeutralMode(NeutralMode.Brake);
        rampMotor.configVoltageCompSaturation(Constants.RAMP_VCOMP);
        rampMotor.enableVoltageCompensation(true);
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
        public double ramp_demand = 0.0;
        public double flap_piston_distance = 0.0;
        public double collector_piston_distance = 0.0;
    }

}
