package frc.robot.actions.buttonactions;

import edu.wpi.first.wpilibj.Joystick;
import frc.lib.statemachine.Action;


public class ButtonWaitAction extends Action {

    private Joystick stick;
    private int button;

    public ButtonWaitAction(Joystick stick, int button) {
        this.stick = stick;
        this.button = button;
    }

    @Override
    public void onStart() {

    }

    @Override
    public void onLoop() {

    }

    @Override
    public boolean isFinished() {
        return stick.getRawButtonPressed(button);
    }

    @Override
    public void onStop() {

    }
}
