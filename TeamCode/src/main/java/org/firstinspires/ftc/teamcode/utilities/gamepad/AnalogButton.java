package org.firstinspires.ftc.teamcode.utilities.gamepad;

import org.firstinspires.ftc.teamcode.utilities.misc.TeleOpMode;

/**
 * Created by LeviG on 1/12/2019.
 */

public abstract class AnalogButton extends Button {

    public AnalogButton(TeleOpMode.Key key) {
        super(new TeleOpMode.Key[]{key});
    }

    public AnalogButton(TeleOpMode.Key[] keys) {
        super(keys);
    }

    @Override
    public void update() {
        this.setOutput(keyValues.get(0));
    }

    public abstract void setOutput(double value);
}