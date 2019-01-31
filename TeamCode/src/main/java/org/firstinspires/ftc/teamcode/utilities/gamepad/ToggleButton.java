package org.firstinspires.ftc.teamcode.utilities.gamepad;

import org.firstinspires.ftc.teamcode.utilities.misc.TeleOpMode;

/**
 * Created by LeviG on 1/12/2019.
 */

public abstract class ToggleButton extends Button {

    private int states;
    private volatile int currentState = 0;
    private volatile boolean isPressed = false;
    private double threshold;

    public ToggleButton(TeleOpMode.Key key) {
        super(new TeleOpMode.Key[]{key});
        this.threshold = 0.5;
        this.states = 2;
    }

    public ToggleButton(TeleOpMode.Key key, double threshold) {
        super(new TeleOpMode.Key[]{key});
        this.threshold = threshold;
        this.states = 2;
    }

    public ToggleButton(TeleOpMode.Key key, int states) {
        super(new TeleOpMode.Key[]{key});
        this.threshold = 0.5;
        this.states = states;
    }

    public ToggleButton(TeleOpMode.Key key, double threshold, int states) {
        super(new TeleOpMode.Key[]{key});
        this.threshold = threshold;
        this.states = states;
    }

    public ToggleButton(TeleOpMode.Key[] keys) {
        super(keys);
        this.threshold = 0.5;
        this.states = 2;
    }

    public ToggleButton(TeleOpMode.Key[] keys, double threshold) {
        super(keys);
        this.threshold = threshold;
        this.states = 2;
    }

    public ToggleButton(TeleOpMode.Key[] keys, int states) {
        super(keys);
        this.threshold = 0.5;
        this.states = states;
    }

    public ToggleButton(TeleOpMode.Key[] keys, double threshold, int states) {
        super(keys);
        this.threshold = threshold;
        this.states = states;
    }

    @Override
    public void update() {
        if(this.keyValues.get(0) > this.threshold && !isPressed) {
            this.currentState += 1;
            if(this.currentState >= this.states) this.currentState = 0;
            isPressed = true;
            this.setOutput(currentState, this.keyValues.get(0));
        } else if ((!(this.keyValues.get(0) > this.threshold)) && isPressed) {
            isPressed = false;
        }
    }

    public int getCurrentState() {
        return currentState;
    }

    public abstract void setOutput(int currentState, double value);

}