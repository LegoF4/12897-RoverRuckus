package org.firstinspires.ftc.teamcode.utilities.gamepad;

import org.firstinspires.ftc.teamcode.utilities.misc.TeleOpMode;

/**
 * Created by LeviG on 1/12/2019.
 */

public abstract class ConditionedDigitalButton extends Button {

    private double threshold;

    public ConditionedDigitalButton(TeleOpMode.Key key) {
        super(new TeleOpMode.Key[]{key});
        threshold = 0.5;
    }

    public ConditionedDigitalButton(TeleOpMode.Key key, double threshold) {
        super(new TeleOpMode.Key[]{key});
        threshold = threshold;
    }

    public ConditionedDigitalButton(TeleOpMode.Key[] keys) {
        super(keys);
        threshold = 0.5;
    }

    public ConditionedDigitalButton(TeleOpMode.Key[] keys, double threshold) {
        super(keys);
        threshold = threshold;
    }

    @Override
    public void update() {
        for (int i = 0; i < keys.length; i++) {
            if(keyValues.get(i) > threshold) {
                this.setOutput(i);
                break;
            }
        }
        setOutput(-1);
    }

    public abstract void setOutput(int state);
}