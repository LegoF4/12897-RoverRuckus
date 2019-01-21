package org.firstinspires.ftc.teamcode.utilities.gamepad;

import org.firstinspires.ftc.teamcode.utilities.misc.TeleOpMode;

/**
 * Created by LeviG on 1/12/2019.
 */

public abstract class DigitalButton extends Button {

    private double threshold;

    public DigitalButton(TeleOpMode.Key key) {
        super(new TeleOpMode.Key[]{key});
        this.threshold = 0.5;
    }

    public DigitalButton(TeleOpMode.Key key, double threshold) {
        super(new TeleOpMode.Key[]{key});
        this.threshold = threshold;
    }

    public DigitalButton(TeleOpMode.Key[] keys) {
        super(keys);
        this.threshold = 0.5;
    }

    public DigitalButton(TeleOpMode.Key[] keys, double threshold) {
        super(keys);
        this.threshold = threshold;
    }

    @Override
    public void update() {
        if(this.keyValues.get(0) > threshold) setOutput();
    }

    public abstract void setOutput();
}