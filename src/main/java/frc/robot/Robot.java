/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.geometry.Rotation2d;
import frc.robot.Drive.DriveControlState;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private Joystick driveStick;
  private Joystick mechStick;

	private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  
  @Override
  public void robotInit() {
    Drive.init();
    PoseEstimator.reset();//reset the pose estimator
    Mech.init();
    Sensors.init();
    driveStick = new Joystick(0);
    mechStick = new Joystick(1);
    Lighting.reset();
    PoseEstimator.init();
    m_chooser.setDefaultOption("Dead Reckon Forward", Constants.kDefaultAuto);//add the auto options to
    m_chooser.addOption("Bottom", Constants.kBottom);
    m_chooser.addOption("Middle", Constants.kMiddle);
    SmartDashboard.putData("Auto choices", m_chooser);
    Drive.driveControlState = DriveControlState.OPEN_LOOP;
  }
  @Override
  public void disabledInit() {
    Lighting.disabled();
  }
  @Override
  public void disabledPeriodic() {
    // Drive.driveLeftTalon.set(0);//reset all motors for saftey, even if first does it for us
    // Drive.driveRightTalon.set(0);
    Mech.resetActuators();
    Auto.reset();
  }
  @Override 
  public void robotPeriodic() {//runs all the time, put sensor and led stuff here
    Sensors.pollSensors();//update sensor data
    Lighting.determineAlliance(m_ds.getAlliance());//make sure the leds are right
    Lighting.ds_attached(m_ds.isDSAttached());
    Lighting.checkMatchEnded(m_ds);
    Lighting.setColor(Sensors.getFieldColor());//show what the field sees
    SmartDashboard.putString("Sensed Color", Sensors.getColor()+"");//put some stuff to the smart dashboard
    SmartDashboard.putString("Field Color", Sensors.getFieldColor()+"");
    SmartDashboard.putString("Field Demand", DriverStation.getInstance().getGameSpecificMessage()+"");
    Auto.createPaths();
  }
  @Override
  public void teleopInit() {
    Lighting.telop();
    Drive.setVomp(true);
  }
  @Override
  public void teleopPeriodic() 
  {
    Drive.drive(driveStick);
    Mech.mech(mechStick);
  }
  static Timer autonTimer = new Timer();
  @Override
  public void autonomousInit() 
  {
    Drive.resetEncoders();//reset encoders for the pose estimator
    PoseEstimator.reset();//reset the pose estimator
    Drive.setVomp(true);
    Sensors.setHeading(Rotation2d.fromDegrees(0));
    Lighting.auto();
    autonTimer.reset();
    autonTimer.start();
    m_autoSelected = m_chooser.getSelected();//Constants.kBottom
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    switch (m_autoSelected) {
      case Constants.kBottom:
        // Auto.autoStream(true);
        Auto.followPath(Auto.bottomPath, false);
        break;
      case Constants.kMiddle:
        // Auto.autoStream(false);
        Auto.followPath(Auto.middlePath, false);
        break;
      case Constants.kDefaultAuto:
      default:
        break;
    }
  }
  @Override
  public void autonomousPeriodic()
  {
    m_autoSelected = m_chooser.getSelected();
    if(m_autoSelected.equals(Constants.kDefaultAuto))
    {
      if(autonTimer.get()<1.5)
      {
        Drive.tank(1, 1);
      }
      else
      {
        Drive.tank(0, 0);
      }
    }
    else
    {
      if(Auto.pathFollowingController!=null)
      {
        Auto.updatePathFollower();
      }
    }
    PoseEstimator.run();
  }
  @Override
  public void testInit() 
  {
    
  }
  @Override
  public void testPeriodic() 
  {
    Drive.test();
  }
}
