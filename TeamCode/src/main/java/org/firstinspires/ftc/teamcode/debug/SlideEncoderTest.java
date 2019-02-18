package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.constructs.Robot;
import org.firstinspires.ftc.teamcode.utilities.misc.LinearOpMode;

/**
 * Created by LeviG on 2/17/2019.
 */
@TeleOp(name = "Slide Encoder Test", group="Debug")
public class SlideEncoderTest extends LinearOpMode {

    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        robot.init();
        waitForStart();
        robot.slides.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(opModeIsActive()) {
            telemetry.addLine("Ticks: " + Integer.toString(robot.slides.getPosition()));
            telemetry.update();
            Thread.sleep(20);
        }
    }

    @Override
    public void stop() {
        robot.stop();
        super.stop();
    }
}
