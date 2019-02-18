package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utilities.gamepad.SingleButton;
import org.firstinspires.ftc.teamcode.utilities.gamepad.ToggleButton;
import org.firstinspires.ftc.teamcode.utilities.misc.TeleOpMode;

/**
 * Created by LeviG on 2/16/2019.
 */
@TeleOp(name = "Servo Tuning", group="Debug")
public class ServoTuning extends TeleOpMode {

    Servo s1;
    Servo s2;
    double s1pos = 0.5;
    double s2pos = 0.5;
    double increment = 0.01;

    @Override
    public void runOpMode() throws InterruptedException {

        s1 = hardwareMap.get(Servo.class, "dl");
        s2 = hardwareMap.get(Servo.class, "dr");

        addButton(new SingleButton(Key.DPAD_UP) {
            @Override
            public void setOutput() {
                s1pos += increment;
            }
        });

        addButton(new SingleButton(Key.DPAD_DOWN) {
            @Override
            public void setOutput() {
                s1pos -= increment;
            }

        });

        addButton(new SingleButton(Key.Y) {
            @Override
            public void setOutput() {
                s2pos += increment;
            }
        });

        addButton(new SingleButton(Key.A) {
            @Override
            public void setOutput() {
                s2pos -= increment;
            }
        });

        addButton(new SingleButton(Key.RIGHT_BUMPER) {
            @Override
            public void setOutput() {
                increment *= 10;
            }
        });

        addButton(new SingleButton(Key.LEFT_BUMPER) {
            @Override
            public void setOutput() {
                increment *= 0.1;
            }
        });
        long startTime = System.currentTimeMillis();
        waitForStart();
        while(opModeIsActive()) {
            updateButtons();
            if(System.currentTimeMillis() > (startTime + 2000)) {
                s1.close();
                s2.close();
            } else {
                telemetry.addLine("S1: " + String.format("%.5f", s1pos));
                telemetry.addLine("S2: " + String.format("%.5f", s2pos));
                telemetry.addLine("Increment: " + String.format("%.5f", increment));
                telemetry.update();
                s1.setPosition(s1pos);
                s2.setPosition(s2pos);
            }
            Thread.sleep(10);
        }
    }

    @Override
    public void stop() {
        super.stop();
    }
}