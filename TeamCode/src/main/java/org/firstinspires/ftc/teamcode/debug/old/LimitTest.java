package org.firstinspires.ftc.teamcode.debug.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.navigation.Position;
import org.firstinspires.ftc.teamcode.utilities.misc.LinearOpMode;

/**
 * Created by LeviG on 2/16/2019.
 */
@TeleOp(name = "Limit Test", group="Debug")
public class LimitTest extends LinearOpMode {

    DigitalChannel digitalChannel;

    @Override
    public void runOpMode() throws InterruptedException {
        digitalChannel = hardwareMap.get(DigitalChannel.class, "limit1");
        digitalChannel.setMode(DigitalChannel.Mode.INPUT);
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addLine("Current State: " +(digitalChannel.getState() ? "NOT DETECTED" : "DETECTED"));
            telemetry.update();
            Thread.sleep(10);
        }
    }
}
