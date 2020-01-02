package frc.robot.actions.buttonactions;

import frc.lib.statemachine.Action;

public class ModAction extends Action {

    Action main, alternate;
    boolean function;
    FunctionSource functionSource;

    public ModAction(Action main, Action alternate, FunctionSource functionSource) {
        this.main = main;
        this.alternate = alternate;
        this.functionSource = functionSource;
    }

    @Override
    public void onStart() {
        function = functionSource.function();
        if (!function) {
            main.onStart();
        } else {
            alternate.onStart();
        }
    }

    @Override
    public void onLoop() {
        if (!function) {
            main.onLoop();
        } else {
            alternate.onLoop();
        }
    }

    @Override
    public boolean isFinished() {
        if (!function) {
            return main.isFinished();
        } else {
            return alternate.isFinished();
        }
    }

    @Override
    public void onStop() {
        if (!function) {
            main.onStop();
        } else {
            alternate.onStop();
        }
    }

    public interface FunctionSource {
        boolean function();
    }
}
