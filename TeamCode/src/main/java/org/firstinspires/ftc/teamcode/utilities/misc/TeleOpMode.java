package org.firstinspires.ftc.teamcode.utilities.misc;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utilities.gamepad.Button;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by LeviG on 1/12/2019.
 */

public abstract class TeleOpMode extends LinearOpMode {

    private volatile List<Button> buttons = new ArrayList<Button>();

    public enum Key {
        LEFT_X,
        LEFT_Y,
        LEFT_STICK,
        RIGHT_X,
        RIGHT_Y,
        RIGHT_STICK,
        A,
        B,
        X,
        Y,
        RIGHT_BUMPER,
        LEFT_BUMPER,
        RIGHT_TRIGGER,
        LEFT_TRIGGER,
        DPAD_UP,
        DPAD_DOWN,
        DPAD_LEFT,
        DPAD_RIGHT
    }

    public void updateButtons() {
        for(Button button : buttons) {
            Key[] keys = button.getKeys();
            List<Double> keyValues = new ArrayList<Double>();
            for (Key key : keys) {
                keyValues.add(getKeyValue(key));
            }
            button.setKeyValues(keyValues);
        }
        for(Button button : buttons) {
            button.update();
        }
    }

    public double getKeyValue(Key key) {
        switch (key) {
            case A:
                return gamepad1.a ? 1 : 0;
            case B:
                return gamepad1.b ? 1 : 0;
            case X:
                return gamepad1.x ? 1 : 0;
            case Y:
                return gamepad1.y ? 1 : 0;
            case LEFT_X:
                return gamepad1.left_stick_x;
            case LEFT_Y:
                return gamepad1.left_stick_y;
            case RIGHT_X:
                return gamepad1.right_stick_x;
            case RIGHT_Y:
                return gamepad1.right_stick_y;
            case LEFT_STICK:
                return gamepad1.left_stick_button ? 1 : 0;
            case RIGHT_STICK:
                return gamepad1.right_stick_button ? 1 : 0;
            case DPAD_UP:
                return gamepad1.dpad_up ? 1 : 0;
            case DPAD_DOWN:
                return gamepad1.dpad_down ? 1 : 0;
            case DPAD_LEFT:
                return gamepad1.dpad_left ? 1 : 0;
            case DPAD_RIGHT:
                return gamepad1.dpad_right ? 1 : 0;
            case LEFT_TRIGGER:
                return gamepad1.left_trigger;
            case RIGHT_TRIGGER:
                return gamepad1.right_trigger;
            case LEFT_BUMPER:
                return gamepad1.left_bumper ? 1 : 0;
            case RIGHT_BUMPER:
                return gamepad1.right_bumper ? 1 : 0;
        }
        return 0;
    }

    public void addButton(Button button) {
        buttons.add(button);
    }

    public boolean removeButton(Button button) {
        return buttons.remove(button);
    }
}