package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.drivers.PIDF;
import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Rotation2d;
import frc.lib.geometry.Translation2d;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.lib.path.AdaptivePurePursuitController;
import frc.lib.path.Lookahead;
import frc.lib.path.Path;
import frc.lib.util.DriveSignal;
import frc.lib.util.HIDHelper;
import frc.lib.util.Util;
import frc.robot.Constants;
import frc.robot.Kinematics;

import java.util.ArrayList;
import java.util.Arrays;

public class Spinny extends Subsystem {

    //construct one and only 1 instance of this class
    private static Spinny m_SpinnyInstance = new Spinny();

    public static Spinny getInstance() {
        return m_SpinnyInstance;
    }

    // Spinner colors in clockwise order
    private ArrayList<String> spinnerColors = new ArrayList<String>(Arrays.asList("B", "G", "R", "Y"));

    //used internally for data
    private SpinnyControlState mSpinnyControlState = SpinnyControlState.INACTIVE;
    private SpinnyIO periodicIO;

    // Hardware
    private TalonSRX spinMotor;
    private ColorSensor colorSensor;

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
                                mSpinnyControlState = SpinnyControlState.INACTIVE;
                                periodicIO.spin_distance = 0.0;
                                periodicIO.activeColor = null;
                            }

                        case AUTO_SPIN:
                            updateEncoding();
                            //TODO
                            break;

                        case AUTO_COLOR:
                            updateEncoding();
                            if (periodicIO.activeTargetColor != null) {

                                // If we're already on target
                                if (periodicIO.activeColor.equals(periodicIO.activeTargetColor)) {
                                    initInactiveEncodingState();
                                }
                                // If the target is counter-clockwise one step
                                else if (spinnerColors.indexOf(periodicIO.activeColor) == spinnerColors.indexOf(periodicIO.activeTargetColor) - 1) {
                                    periodicIO.spin_demand = Constants.AUTO_COLOR_BACKWARD_SPEED;
                                }
                                // If the target is clockwise one or two steps
                                else {
                                    periodicIO.spin_demand = Constants.AUTO_COLOR_FORWARD_SPEED;
                                }
                            }
                            break;

                        case MANUAL:
                            updateEncoding();
                            if (periodicIO.forwardButtonPressed) {
                                periodicIO.spin_demand = Constants.PERCENT_MANUAL_FORWARD;
                            }
                            else if (periodicIO.backwardButtonPressed) {
                                periodicIO.spin_demand = Constants.PERCENT_MANUAL_BACKWARD;
                            }
                            else {
                                initInactiveEncodingState();
                            }
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

    private void initInactiveEncodingState() {
        mSpinnyControlState = SpinnyControlState.INACTIVE_ENCODING;
    }

    private void updateEncoding() {
        rawColorData = getRawColorData;
        String color = resolveToColor(rawColorData.red, rawColorData.green, rawColorData.blue);
        if (color != null && !color.equals(periodicIO.activeColor)) {
            if (periodicIO.activeColor != null) {
                periodicIO.spin_distance += 0.125;
            }
            periodicIO.activeColor = color;
        }
    }

    public void initAutoColor() {
        String targetColor = DriverStation.getInstance().getGameSpecificMessage();
        if (!spinnerColors.contains(targetColor))
            return;
        periodicIO.activeTargetColor = spinnerColors.get((spinnerColors.indexOf(targetColor) + 2) % 4);
        mSpinnyControlState = SpinnyControlState.AUTO_COLOR;
    }

    public void abort() {
        mSpinnyControlState = SpinnyControlState.INACTIVE_ENCODING;
        periodicIO.startTimestamp = Timer.getFPGATimestamp();

    }

    @Override
    public synchronized void readPeriodicInputs() {
        //TODO read in button info and color sensor data
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        spinMotor.set(ControlMode.PercentOutput, periodicIO.spin_demand);
    }

    private Spinny() {
        spinMotor = new TalonSRX(Constants.SPINNY_ID);
        configTalons();
        reset();

    }

    public void reset() {
        periodicIO = new Spinny.SpinnyIO();
        abort();
    }

    private void configTalons() {
        ErrorCode sensorPresent;
        sensorPresent = spinMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100); //primary closed-loop, 100 ms timeout
        if (sensorPresent != ErrorCode.OK) {
            DriverStation.reportError("Could not detect left encoder: " + sensorPresent, false);
        }
        spinMotor.setSensorPhase(true);
        spinMotor.selectProfileSlot(0, 0);
        spinMotor.config_kF(0, Constants.SPINNY_KF, 0);
        spinMotor.config_kP(0, Constants.SPINNY_KP, 0);
        spinMotor.config_kI(0, Constants.SPINNY_KI, 0);
        spinMotor.config_kD(0, Constants.SPINNY_KD, 0);
        spinMotor.config_IntegralZone(0, 300);
        spinMotor.setInverted(false);
        spinMotor.setNeutralMode(NeutralMode.Brake);
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
        public String activeTargetColor = null;

        // TODO add color sensor input

        // Operator input
        public boolean forwardButtonPressed = false;
        public boolean backwardButtonPressed = false;

        // OUTPUTS
        // Internal counters
        public String activeColor = null;
        public double startTimestamp = 0.0;

        // Spin motor output values
        public double spin_demand = 0.0;
        public double spin_distance = 0.0;
    }

    /**
     * internal methods beyond this point
     **/

    private static int compute_distance(int[] target, int red, int green, int blue) {
        int distance_r = Math.abs(target[0] - red);
        int distance_g = Math.abs(target[1] - green);
        int distance_b = Math.abs(target[2] - blue);

        return distance_r + distance_g + distance_b;
    }

    private static String resolveToColor(int red, int blue, int green) {
        int distance_blue = compute_distance(Constants.BLUE_IDEAL_COLOR_READINGS, red, green, blue);
        int distance_green = compute_distance(Constants.GREEN_IDEAL_COLOR_READINGS, red, green, blue);
        int distance_red = compute_distance(Constants.RED_IDEAL_COLOR_READINGS, red, green, blue);
        int distance_yellow = compute_distance(Constants.YELLOW_IDEAL_COLOR_READINGS, red, green, blue);

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
        return rotations * 32 / 3;
    }

    private static double wheelRotationToCompliantRotation(double rotations) {
        return rotations * 3 / 32;
    }

    private static double rotationsToInches(double rotations) {
        return rotations * Math.PI * Constants.DRIVE_WHEEL_DIAMETER_INCHES;
    }

    private static double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }

    private static double inchesToRotations(double inches) {
        return inches / (Math.PI * Constants.DRIVE_WHEEL_DIAMETER_INCHES);
    }

    private static double inchesPerSecondToRpm(double inches_per_second) {
        return inchesToRotations(inches_per_second) * 60;
    }

    private static double radiansPerSecondToTicksPer100ms(double rad_s) {
        return rad_s / (Math.PI * 2.0) * 4096.0 / 10.0;
    }

    private static double inchesPerSecondToRadiansPerSecond(double in_sec) {
        return in_sec / (Constants.DRIVE_WHEEL_DIAMETER_INCHES * Math.PI) * 2 * Math.PI;
    }

    private static double rpmToTicksPer100ms(double rpm) {
        return ((rpm * 512.0) / 75.0);
    }
}
