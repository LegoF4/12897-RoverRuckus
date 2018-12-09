package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.utilities.misc.LinearOpMode;

/**
 * Created by LeviG on 12/9/2018.
 */
@Autonomous(name = "Basic Encoder Test")
public class BasicEncoderTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
            AnalogInput left = hardwareMap.get(AnalogInput.class,"left");
            AnalogInput center = hardwareMap.get(AnalogInput.class,"center");
        AnalogInput right = hardwareMap.get(AnalogInput.class,"right");
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addLine("Left: " + Double.toString(left.getVoltage()));
            telemetry.addLine("Center: " + Double.toString(center.getVoltage()));
            telemetry.addLine("Right: " + Double.toString(right.getVoltage()));
            telemetry.update();
            Thread.sleep(50);
        }
    }
}
