package org.firstinspires.ftc.teamcode.utilities.gamepad;

import org.firstinspires.ftc.teamcode.utilities.misc.TeleOpMode;

/**
 * Created by LeviG on 1/12/2019.
 */

public abstract class ConditionedAnalogButton extends Button {

    private double threshold = 0.5;

    public ConditionedAnalogButton(TeleOpMode.Key key) {
        super(new TeleOpMode.Key[]{key});
    }

    public ConditionedAnalogButton(TeleOpMode.Key[] keys) {
        super(keys);
    }

    @Override
    public void update() {
        for (int i = 0; i < keys.length; i++) {
            if(keyValues.get(i) > threshold) {
                this.setOutput(i, keyValues.get(i));
                break;
            }
        }
        setOutput(-1, 0.0);
    }

    public abstract void setOutput(int state, double value);
}