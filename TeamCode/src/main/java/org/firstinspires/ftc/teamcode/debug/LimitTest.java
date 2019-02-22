package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.utilities.misc.LinearOpMode;

/**
 * Created by LeviG on 2/16/2019.
 */
@TeleOp(name = "Limit Test", group="Debug")
public class LimitTest extends LinearOpMode {

    DigitalChannel downLimit;
    DigitalChannel upLimit;

    @Override
    public void runOpMode() throws InterruptedException {
        downLimit = hardwareMap.get(DigitalChannel.class, "limit1");
        downLimit.setMode(DigitalChannel.Mode.INPUT);
        upLimit = hardwareMap.get(DigitalChannel.class, "liftLimit");
        upLimit.setMode(DigitalChannel.Mode.INPUT);
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addLine("Down Limit: " +(downLimit.getState() ? "NOT DETECTED" : "DETECTED"));
            telemetry.addLine("Up Limit: " +(upLimit.getState() ? "NOT DETECTED" : "DETECTED"));
            telemetry.update();
            Thread.sleep(10);
        }
    }
}
