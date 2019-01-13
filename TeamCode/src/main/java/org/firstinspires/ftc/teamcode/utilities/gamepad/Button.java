package org.firstinspires.ftc.teamcode.utilities.gamepad;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utilities.misc.TeleOpMode;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by LeviG on 1/12/2019.
 */

public abstract class Button {

    protected TeleOpMode.Key[] keys;
    protected List<Double> keyValues;

    public Button(TeleOpMode.Key[] keys) {
        this.keys = keys;
        setKeyValues(new ArrayList<Double>());
    }

    public Button() {
        keys = new TeleOpMode.Key[0];
        keyValues = new ArrayList<Double>();
    }

    public TeleOpMode.Key[] getKeys() {
        return keys;
    }

    public void setKeyValues(List keyValues) {
        while(keyValues.size() < keys.length) {
            keyValues.add(0.0d);
        }
        this.keyValues = keyValues;
    }

    public abstract void update();
}