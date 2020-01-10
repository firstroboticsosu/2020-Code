package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.drivers.ColorSensor;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.lib.motion.MotionProfileConstraints;
import frc.lib.motion.MotionProfileGoal;
import frc.lib.motion.MotionState;
import frc.lib.motion.ProfileFollower;
import frc.robot.Constants;

import java.util.ArrayList;
import java.util.Arrays;

public class Spinny extends Subsystem {

    //construct one and only 1 instance of this class
    private static Spinny m_SpinnyInstance = new Spinny();

    public static Spinny getInstance() {
        return m_SpinnyInstance;
    }

    // Spinner colors in clockwise order
    private ArrayList<String> spinnerColors = new ArrayList<>(Arrays.asList("B", "G", "R", "Y"));

    //used internally for data
    private SpinnyControlState mSpinnyControlState = SpinnyControlState.INACTIVE;
    private SpinnyIO periodicIO;
    private ProfileFollower velocityFollower;
    private MotionProfileConstraints motionConstraints;

    // Hardware
    private TalonSRX spinMotor;
    private ColorSensor colorSensor;
    private DoubleSolenoid deployPistonSolenoid;

    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Spinny.this) {

                    switch (mSpinnyControlState) {

                        case INACTIVE:
                            periodicIO.spin_demand = 0;
                            break;

                        case INACTIVE_ENCODING:
                            updateEncoding();
                            if (timestamp - periodicIO.startTimestamp >= Constants.INACTIVE_ENCODING_STATE_TIME) {
                                // Move to INACTIVE state
                                mSpinnyControlState = SpinnyControlState.INACTIVE;
                                periodicIO.observed_rotation = 0.0;
                                periodicIO.deployColor = false;
                                periodicIO.activeColor = null;
                            }
                            break;

                        case AUTO_SPIN:
                            updateEncoding();
                            updateAutoSpin(timestamp);
                            break;

                        case AUTO_COLOR:
                            updateEncoding();
                            updateAutoColor();
                            break;

                        case MANUAL:
                            updateEncoding();
                            break;

                        default:
                            System.out.println("Unexpected control state");
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
        periodicIO.sensedColors = colorSensor.getColor();
        periodicIO.red = periodicIO.sensedColors[0];
        periodicIO.green = periodicIO.sensedColors[1];
        periodicIO.blue = periodicIO.sensedColors[2];
        periodicIO.encoder_distance = spinMotor.getSelectedSensorPosition();
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if(mSpinnyControlState != SpinnyControlState.AUTO_SPIN || mSpinnyControlState != SpinnyControlState.AUTO_COLOR){
            spinMotor.set(ControlMode.Velocity, periodicIO.spin_demand);
        } else {
            spinMotor.set(ControlMode.PercentOutput, periodicIO.spin_demand);
        }
        deployPistonSolenoid.set(periodicIO.deployColor ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward);
    }

    private Spinny() {
        spinMotor = new TalonSRX(Constants.SPINNY_ID);
        deployPistonSolenoid = new DoubleSolenoid(Constants.COLOR_IN_ID, Constants.COLOR_OUT_ID);
        colorSensor = new ColorSensor();
        motionConstraints = new MotionProfileConstraints(Constants.SPINNY_MAX_VEL, Constants.SPINNY_MAX_ACCEL);
        velocityFollower = new ProfileFollower(Constants.SPINNY_KP, Constants.SPINNY_KI, Constants.SPINNY_KV,
                Constants.SPINNY_KFFV, Constants.SPINNY_KFFA, Constants.SPINNY_KS);
        configTalons();
        reset();

    }

    public void reset() {
        periodicIO = new Spinny.SpinnyIO();
        velocityFollower.setConstraints(motionConstraints);
        velocityFollower.resetIntegral();
        velocityFollower.resetProfile();
        velocityFollower.resetSetpoint();
        resetSensors();
        abort();
    }

    public void resetSensors(){
        colorSensor.reset();
        spinMotor.setSelectedSensorPosition(0, 0, 0);
    }

    private void initInactiveEncodingState() {
        periodicIO.startTimestamp = Timer.getFPGATimestamp();
        mSpinnyControlState = SpinnyControlState.INACTIVE_ENCODING;
        periodicIO.spin_demand = 0; // Just to make sure
    }

    private void updateEncoding() {
        //want piston deployed if the color is being read
        periodicIO.deployColor = true; 
        int[] colorData = {(int) (255*periodicIO.sensedColors[0]),
            (int) (255*periodicIO.sensedColors[1]), (int) (255*periodicIO.sensedColors[2])};
        //resolve data to a color string
        String color = resolveToColor(colorData);

        //if the color has changed, update the observed spin distance
        if (color != null && !color.equals(periodicIO.activeColor)) {
            if (periodicIO.activeColor != null) {
                periodicIO.observed_rotation += 0.125; // quarter of a rotation
            }
            periodicIO.activeColor = color;
        }
    }

    public void initAutoColor() {
        String targetColor = DriverStation.getInstance().getGameSpecificMessage();
        if (!spinnerColors.contains(targetColor)) return;
        periodicIO.activeTargetColor = spinnerColors.get((spinnerColors.indexOf(targetColor) + 2) % 4);
        mSpinnyControlState = SpinnyControlState.AUTO_COLOR;
    }

    private void updateAutoColor(){
        if (periodicIO.activeTargetColor != null) {

            // If we're already on target
            if (periodicIO.activeColor.equals(periodicIO.activeTargetColor)) {
                periodicIO.activeTargetColor = null;
                initInactiveEncodingState();
            }
            // If the target is counter-clockwise one or two steps
            else if (spinnerColors.indexOf(periodicIO.activeColor) < spinnerColors.indexOf(periodicIO.activeTargetColor)) {
                periodicIO.spin_demand = Constants.AUTO_COLOR_BACKWARD_SPEED;
            }
            // If the target is clockwise one or two steps
            else {
                periodicIO.spin_demand = Constants.AUTO_COLOR_FORWARD_SPEED;
            }
        }
    }

    private void initFancyAutoColor(){
        //get the color from the FMS
        String targetColor = DriverStation.getInstance().getGameSpecificMessage();
        if (!spinnerColors.contains(targetColor)) return;
        periodicIO.activeTargetColor = spinnerColors.get((spinnerColors.indexOf(targetColor) + 2) % 4);

        if (mSpinnyControlState != SpinnyControlState.AUTO_SPIN) {
            spinMotor.set(ControlMode.Velocity, 0);         
        }
        velocityFollower.resetIntegral();
        velocityFollower.resetProfile();
        velocityFollower.resetSetpoint();
        velocityFollower.setGoal(new MotionProfileGoal(30 * Math.PI * 3)); // rotate the wheel n inches to the desired color
    
        mSpinnyControlState = SpinnyControlState.AUTO_SPIN;
    }

    public void initAutoSpin(){
        if (mSpinnyControlState != SpinnyControlState.AUTO_SPIN) {
            spinMotor.set(ControlMode.Velocity, 0);         
        }
        velocityFollower.resetIntegral();
        velocityFollower.resetProfile();
        velocityFollower.resetSetpoint();
        velocityFollower.setGoal(new MotionProfileGoal(30 * Math.PI * 3)); // rotate the wheel 3 rotations (30 in diameter and the result is the arc length)
        mSpinnyControlState = SpinnyControlState.AUTO_SPIN;
    }

    private void updateAutoSpin(double timestamp){
        //create the new motion state of the system
        final MotionState cur_state = new MotionState(timestamp, ticksToInches(periodicIO.encoder_distance),
         ticksPer100msToInPerSecond(periodicIO.encoder_velocity), 0.0);
        //calculate the next update to the velocity
        periodicIO.spin_demand = inchesPerSecondToTicksPer100ms(velocityFollower.update(cur_state, timestamp));
    }

    public boolean autoSpinComplete(){
        return velocityFollower.onTarget();
    }

    public void updateManualSpin(double speed){
        if(mSpinnyControlState != SpinnyControlState.MANUAL){
            spinMotor.set(ControlMode.PercentOutput, 0);
            mSpinnyControlState = SpinnyControlState.MANUAL;
        }
        periodicIO.spin_demand = speed;
    }

    public void deactivate(){
        initInactiveEncodingState();
    }

    public void abort() {
        mSpinnyControlState = SpinnyControlState.INACTIVE;
    }

    public void setColorDeploy(boolean deployed){
        periodicIO.deployColor = deployed;
    }

    public boolean getColorDeployed(){
        return periodicIO.deployColor;
    }

    private void configTalons() {
        ErrorCode sensorPresent;
        sensorPresent = spinMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100); //primary closed-loop, 100 ms timeout
        if (sensorPresent != ErrorCode.OK) {
            DriverStation.reportError("Could not detect spinner encoder: " + sensorPresent, false);
        }
        spinMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);
        spinMotor.setSensorPhase(true);
        spinMotor.setInverted(false);
        spinMotor.setNeutralMode(NeutralMode.Coast);
        spinMotor.configVoltageCompSaturation(Constants.SPINNY_VCOMP);
        spinMotor.enableVoltageCompensation(true);
    }

    @Override
    public PeriodicIO getLogger() {
        return periodicIO;
    }

    public void onStop(){

    }

    enum SpinnyControlState {
        INACTIVE,
        INACTIVE_ENCODING,
        MANUAL,
        AUTO_SPIN,
        AUTO_COLOR;

        public String toString() {
            return name().charAt(0) + name().substring(1).toLowerCase();
        }
    }

    public void outputTelemetry() {
        SmartDashboard.putString("Drive/Drive State", mSpinnyControlState.toString());
    }

    public class SpinnyIO extends PeriodicIO {
        // INPUTS
        public String activeTargetColor = null; // The color we need to see to have the correct color showing on the sensor
        public double[] sensedColors = {0, 0, 0}; // The input we get from the sensor
        public double red = 0, green = 0, blue = 0;

        public double observed_rotation = 0.0; // How much we've spun the wheel recently
        public double encoder_velocity = 0.0;
        public double encoder_distance = 0.0;

        // OUTPUTS
        // Internal counters
        public String activeColor = null; // What the color we're seeing right now is, null for unknown
        public double startTimestamp = 0.0; // The time at which we disengaged an active mode, used to disable vision encoding after time

        // Spin motor output values
        public double spin_demand = 0.0; // The outgoing request for spin speed
        public boolean deployColor = false;
    }

    /**
     * internal methods beyond this point
     **/

    private static int compute_distance(int[] target, int[] colors) {
        int distance_r = Math.abs(target[0] - colors[0]);
        int distance_g = Math.abs(target[1] - colors[1]);
        int distance_b = Math.abs(target[2] - colors[2]);

        return distance_r + distance_g + distance_b;
    }

    private static String resolveToColor(int[] colors) {
        int distance_blue = compute_distance(Constants.BLUE_IDEAL_COLOR_READINGS, colors);
        int distance_green = compute_distance(Constants.GREEN_IDEAL_COLOR_READINGS, colors);
        int distance_red = compute_distance(Constants.RED_IDEAL_COLOR_READINGS, colors);
        int distance_yellow = compute_distance(Constants.YELLOW_IDEAL_COLOR_READINGS, colors);

        int smallestDistance = Math.min(Math.min(distance_blue, distance_green), Math.min(distance_red, distance_yellow));

        if (smallestDistance > Constants.MAXIMUM_TOLERANCE) {
            return null;
        }
        else if (distance_blue == smallestDistance) {
            return "B";
        }
        else if (distance_green == smallestDistance) {
            return "G";
        }
        else if (distance_red == smallestDistance) {
            return "R";
        }
        else {
            return "Y";
        }
    }

    private static double compliantRotationToWheelRotation(double rotations) {
        return rotations * 30 / 3;
    }

    private static double wheelRotationToCompliantRotation(double rotations) {
        return rotations * 3 / 30;
    }

    private double ticksPer100msToInPerSecond(double ticks_per_100ms) {
        return ticksToInches(ticks_per_100ms) * 10.0;
    }

    private double inchesPerSecondToTicksPer100ms(double inches_per_second) {
        return inchesToTicks(inches_per_second) / 10.0;
    }

    private double inchesToTicks(double inches) {
        return inches * Math.PI * Constants.SPINNY_WHEEL_DIAMETER;
    }

    protected double ticksToInches(double ticks) {
        return ticks / (Math.PI * Constants.SPINNY_WHEEL_DIAMETER);
    }

}
