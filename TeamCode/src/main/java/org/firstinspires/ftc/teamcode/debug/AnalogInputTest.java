package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.misc.LinearOpMode;

/**
 * Created by LeviG on 10/28/2018.
 */
@TeleOp(name = "Analog Input Test")
public class AnalogInputTest extends LinearOpMode {

    AnalogInput inputC;
    AnalogInput inputL;
    AnalogInput inputR;

    @Override
    public void runOpMode() throws InterruptedException{
        inputC = hardwareMap.get(AnalogInput.class, "center");
        inputL = hardwareMap.get(AnalogInput.class, "left");
        inputR = hardwareMap.get(AnalogInput.class, "right");
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addLine("Left: " + Double.toString(inputL.getVoltage()));
            telemetry.addLine("Center: " + Double.toString(inputC.getVoltage()));
            telemetry.addLine("Right: " + Double.toString(inputR.getVoltage()));
            telemetry.update();
            Thread.sleep(50);
        }
    }
}
