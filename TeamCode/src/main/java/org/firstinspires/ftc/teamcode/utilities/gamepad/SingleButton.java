package org.firstinspires.ftc.teamcode.utilities.gamepad;

import org.firstinspires.ftc.teamcode.utilities.misc.TeleOpMode;

/**
 * Created by LeviG on 2/16/2019.
 */

public abstract class SingleButton extends Button {

    private volatile boolean isPressed = false;

    public SingleButton(TeleOpMode.Key key) {
        super(new TeleOpMode.Key[]{key});
    }

    public SingleButton(TeleOpMode.Key[] keys) {
        super(keys);
    }

    @Override
    public void update() {
        if(this.keyValues.get(0) > 0.5 && !isPressed) {
            isPressed = true;
            setOutput();
        } else if (this.keyValues.get(0) < 0.5 && isPressed) {
            isPressed = false;
        }
    }

    public abstract void setOutput();
}
