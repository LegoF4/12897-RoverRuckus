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

    AnalogInput input;

    @Override
    public void runOpMode() throws InterruptedException{
        input = hardwareMap.get(AnalogInput.class, "center");
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addLine("Value: " + Double.toString(input.getVoltage()));
            telemetry.update();
            Thread.sleep(50);
        }
    }
}
