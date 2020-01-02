package frc.robot.statemachines;

import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.driveactions.DriveTra;
import frc.robot.planners.DriveTrajectoryGenerator;

public class TestMach extends StateMachineDescriptor{
    
    public TestMach(){
        addSequential(new DriveTra(DriveTrajectoryGenerator.getInstance().getTestCurve()), 10000);
    }

}