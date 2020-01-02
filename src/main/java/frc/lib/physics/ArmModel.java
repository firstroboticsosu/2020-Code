package frc.lib.physics;

import java.text.DecimalFormat;

import frc.lib.geometry.Rotation2d;
import frc.lib.geometry.Translation2d;
import frc.lib.util.CSVWritable;

public class ArmModel {

    // Equivalent moment of inertia when accelerating purely angularly, in kg*m^2.
    protected final double prox_moi_;
    protected final double dist_moi_;

    // Equivalent length of each segment, in m
    protected final double prox_len_;
    protected final double dist_len_;

    // Transmissions for both joints of the arm.
    protected final DCMotorTransmission prox_transmission_;
    protected final DCMotorTransmission dist_transmission_;

    /*TODO Nuke most of this code and solve using linear algerbra and a small bit of trig
    should be able to apply rotation 2d's to do some of the heavy basis shifting
    most of this has to do with the inverse kinematics
    the forward kinematics should be okay*/


    public ArmModel(double prox_len, double dist_len, double prox_moi, double dist_moi, DCMotorTransmission prox_transmission, DCMotorTransmission dist_transmission) {
        this.prox_len_ = prox_len;
        this.dist_len_ = dist_len;
        this.prox_moi_ = prox_moi;
        this.dist_moi_ = dist_moi;
        this.prox_transmission_ = prox_transmission;
        this.dist_transmission_ = dist_transmission;
    }

    /**
     * Generates an X Y coordinate for the end effector based on two angles.
     * Can be angular position, velocity or acceleration. Uses a vector based
     * representation for calculation. NOTE: domains are reversed though due to having 
     * 0 degrees as vertical
     *
     * @param prox angle of the proximal joint
     * @param dist angle of the distal joint
     * @return translation 2d containing end effector position
     */
    public Translation2d solveForwardKinematics(Rotation2d prox, Rotation2d dist) {
        double x = (prox.sin() * prox_len_) + (dist.sin() * dist_len_); // Domains are reversed due to 0 being perfectly vertical
        double y = (prox.sin() * prox_len_) + (dist.cos() * dist_len_);
        return new Translation2d(x, y);
    }


    //TODO nuke this
    /**
     * Generates a set of angles for an xy coordinate
     * <p>Can be position, velocity or acceleration
     *
     * @param x X coordinate in m
     * @param y Y coordinate in m
     * @return a position based arm state
     */
    public ArmState solveInverseKinematics(double x, double y) {
        //find rotation offset into new basis
        //calculate the distance between (0,0) and (x,y)
        //solve for distal angle with distance
        //solve for proximal angle with distance
        //add accumulate rotation offset into proximal angle to get the actual angle
        double distTheta = Math.acos((x * x + y * y
                - prox_len_ * prox_len_ - dist_len_ * dist_len_) / (2 * dist_len_ * prox_len_));
        double proxTheta = Math.atan2(y, x) - Math.atan2(dist_len_ * Math.sin(distTheta),
                prox_len_ + dist_len_ * Math.cos(distTheta));
        distTheta += proxTheta;
        return new ArmState(proxTheta, distTheta);
    }

    /**
     * function to determine if the arm state falls in a possible
     * state of interference with itself or known geometry.
     * @return
     */
    public boolean checkForInterference(){
        //TODO implement this
        return false;
    }

    public ArmDynamics solveForwardDynamics(ArmState angularPosition, ArmState Voltage, ArmState AngularVelocity) {
        ArmDynamics output = new ArmDynamics();
        //TODO manipulate output
        return output;
    }

    public ArmDynamics solveInverseDynamics(ArmState angularPosition, ArmState angularVelocity, ArmState angularAcceleration) {
        ArmDynamics output = new ArmDynamics();
        //TODO manipulate output
        return output;

    }

    // Can refer to position, velocity, acceleration, torque, voltage, etc., depending on context.
    public static class ArmState {
        public double prox = 0.0;
        public double dist = 0.0;

        /**
         * The current state of a 2 axis arm. 
         * This can represent position, velocity, torque, voltage, etc depending on context
         * @param proximal value
         * @param distal value 
         */
        public ArmState(double prox, double dist) {
            this.prox = prox;
            this.dist = dist;
        }

        public ArmState() {
        }

        public double getProximal() {
            return prox;
        }

        public double getDistal(){
            return dist;
        }

        public void setProximal(double val) {
            prox = val;
        }

        public void setDistal(double val){
            dist = val;
        }

        @Override
        public String toString() {
            DecimalFormat fmt = new DecimalFormat("#0.000");
            return fmt.format(prox) + ", " + fmt.format(dist);
        }

        public boolean isNAN(){
            return prox == Double.NaN || dist == Double.NaN;
        }
    }

    //full state dynamics of a 2 axis arm
    public static class ArmDynamics implements CSVWritable {
        public ArmState angular_position = new ArmState(); // rad
        public ArmState angular_velocity = new ArmState(); // rad/s
        public ArmState angular_acceleration = new ArmState(); // rad/s^2
        public ArmState voltage = new ArmState(); // V
        public ArmState torque = new ArmState(); // N m

        public String toCSV() {
            return angular_velocity + ", " + angular_acceleration + ", " + voltage + ", " + torque;
        }

        @Override
        public String toString() {
            return "omega:(" + angular_velocity + ") alpha:(" + angular_acceleration + ") V:(" + voltage + ") tau:(" + torque + ")";
        }
    }

}
