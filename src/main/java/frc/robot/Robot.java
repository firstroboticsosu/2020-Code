/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Arrays;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.loops.Looper;
import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachine;
import frc.lib.util.VersionData;
import frc.robot.actions.spinny.ManualSpin;
import frc.robot.statemachines.TestMach;
import frc.robot.subsystems.*;

/**
 * The VM is configured to automatically run this class. If you change the name
 * of this class or the package after creating this project, you must also
 * update the build.gradle file in the project.
 */
public class Robot extends TimedRobot {

    JoystickButton spinLeft, spinRight, openDoor, closeDoor,
     deployRoller, RetractRoller, deployColor, retractColor, intake;

    SubsystemManager manager;
    Looper enabledLooper;
    Looper disabledLooper;

    public void robotInit() {
        //Create instance of subsystem manager
        this.manager = new SubsystemManager(Arrays.asList(
            PoseEstimator.getInstance(),
            Drive.getInstance()
            ), true);

        //Create instance of loopers for enabled and disabled modes
        this.enabledLooper = new Looper();
        this.disabledLooper = new Looper();

        //Register looper instances to the manager
        this.manager.registerEnabledLoops(this.enabledLooper);
        this.manager.registerDisabledLoops(this.disabledLooper);

        //Final cleanup after initialization
        VersionData.WriteBuildInfoToDashboard();
        Lighting.reset();
    }

    @Override
    public void robotPeriodic() {
        Drive.getInstance().outputTelemetry();
        Lighting.getInstance().determineAlliance(m_ds.getAlliance());
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        //Stop current looper and start new looper
        enabledLooper.stop();
        StateMachine.assertStop();

        //reset any subsystems
        //not normally neccessary for most systems

        //start new looper
        disabledLooper.start();
        Lighting.getInstance().disabled();
    }

    @Override
    public void disabledPeriodic() {
        
    }

    @Override
    public void autonomousInit() {
        //Stop current looper and start new looper
        disabledLooper.stop();
        
        //reset any subsystems
        Drive.getInstance().reset();
        PoseEstimator.getInstance().reset();

        //start new looper
        enabledLooper.start();
        Lighting.getInstance().auto();

        //start the state machine
        StateMachine.runMachine(new TestMach());
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        //Stop current looper and start new looper
        disabledLooper.stop();
        //reset any subsystems
        Drive.getInstance().reset();
        PoseEstimator.getInstance().reset();

        //start new looper
        enabledLooper.start();

        //Start scheduler
        Lighting.getInstance().telop();
    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void testInit() {
        //Stop current looper
        disabledLooper.stop();
        
        //reset any subsystems
        Drive.getInstance().reset();
        PoseEstimator.getInstance().reset();

        //start new looper
        enabledLooper.start();
    }

    @Override
    public void testPeriodic() {

    }

    //TODO make these global? seems to be yes
    public void initButtons(){
        spinLeft = new JoystickButton(Constants.MASTER, 1);
        spinLeft.whileHeld(Action.toCommand2(new ManualSpin(0.5)));

        spinRight = new JoystickButton(Constants.MASTER, 2);

        openDoor = new JoystickButton(Constants.MASTER, 3);

        closeDoor = new JoystickButton(Constants.MASTER, 4);

        deployRoller = new JoystickButton(Constants.MASTER, 5);

        RetractRoller = new JoystickButton(Constants.MASTER, 6);

        deployColor = new JoystickButton(Constants.MASTER, 7);

        retractColor = new JoystickButton(Constants.MASTER, 8);

        intake = new JoystickButton(Constants.MASTER, 9);

    }

}
