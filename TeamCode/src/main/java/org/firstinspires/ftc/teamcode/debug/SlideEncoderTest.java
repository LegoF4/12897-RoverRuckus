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
        robot.setDeposit(Robot.Deposit.HIGH);
        waitForStart();
        robot.slides.prepForEncoders();
        int startPos = robot.slides.getPosition();
        robot.slides.setTargetPosition(robot.slides.getPosition() - 850);
        robot.slides.setPower(0.85);
        long slideTime = System.currentTimeMillis();
        while(robot.slides.getPosition() > (startPos - 850) && opModeIsActive() && System.currentTimeMillis() < (slideTime + 2200)) {
            Thread.sleep(20);
        }
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
