package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constructs.DriveTrain;
import org.firstinspires.ftc.teamcode.constructs.Robot;
import org.firstinspires.ftc.teamcode.utilities.misc.LinearOpMode;
import org.firstinspires.ftc.teamcode.utilities.misc.StaticLog;

/**
 * Created by LeviG on 12/11/2018.
 */
@TeleOp(name = "kV Test")
public class kVTest extends LinearOpMode {

    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException{
        StaticLog.clearLog();
        robot = new Robot(hardwareMap);
        robot.init();
        waitForStart();
        long startTime = System.currentTimeMillis();

        double increment = 0.001;

        double kV = 0;
        double currentX;
        long currentTime;

        boolean incrementUp = false;
        boolean incrementDown = false;
        boolean increase = false;
        boolean decrease = false;
        robot.driveTrain.startOdometry();
        Thread.sleep(300);
        double previousX = robot.driveTrain.getPosition().x;
        long previousTime = System.currentTimeMillis();
        while(opModeIsActive()) {
            if(gamepad1.right_bumper && !incrementUp) {
                incrementUp = true;
                increment *= 10;
            } else if (incrementUp && !gamepad1.right_bumper) {
                incrementUp = false;
            } else if(gamepad1.left_bumper && !incrementDown) {
                incrementDown = true;
                increment *= 1/10;
            } else if (incrementDown && !gamepad1.left_bumper) {
                incrementDown = false;
            }

            if(gamepad1.dpad_up && !increase) {
                increase = true;
            } else if (increase && !gamepad1.dpad_up) {
                increase = false;
                kV += increment;
            } else if(gamepad1.dpad_down && !decrease) {
                decrease = true;
                kV -= increment;
            } else if (decrease && !gamepad1.dpad_down) {
                decrease = false;
            }
            if(gamepad1.left_stick_y > 0.8) robot.driveTrain.setPower(kV* DriveTrain.maxVelocity);
            else if (gamepad1.left_stick_y < -0.8) robot.driveTrain.setPower(kV* DriveTrain.maxVelocity*-1);
            else robot.driveTrain.setPower(kV*DriveTrain.maxVelocity*gamepad1.left_stick_y);

            currentX = robot.driveTrain.getPosition().x;
            currentTime = System.currentTimeMillis();
            telemetry.addLine("Velocity: " + String.format("%.6f", (1000*(currentX-previousX))/(currentTime-previousTime)));
            telemetry.addLine("kV: " + String.format("%.6f", kV));
            telemetry.addLine("Increment: " + Double.toString(increment));
            telemetry.update();
            Thread.sleep(50);
            previousTime = currentTime;
            previousX = currentX;
        }
    }

    @Override
    public void stop() {
        robot.stop();
        super.stop();
    }
}
