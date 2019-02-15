package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constructs.Robot;
import org.firstinspires.ftc.teamcode.controllers.ControllerPID;
import org.firstinspires.ftc.teamcode.utilities.misc.LinearOpMode;
import org.firstinspires.ftc.teamcode.utilities.misc.StaticLog;

/**
 * Created by LeviG on 12/11/2018.
 */
@TeleOp(name = "PID Driving")
public class TeleOpPIDDriving extends LinearOpMode {

    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException{
        StaticLog.clearLog();
        robot = new Robot(hardwareMap);
        robot.init();
        waitForStart();
        long startTime = System.currentTimeMillis();

        double increment = 0.001;
        int index = 0;

        double kP = 0.02;
        double kI = 0;
        double kD = 0;
        double kA = 0;
        double kV = 0;
        double power = 0.8;

        boolean incrementUp = false;
        boolean incrementDown = false;
        boolean increase = false;
        boolean decrease = false;
        boolean previous = false;
        boolean next = false;
        boolean trigger = false;
        boolean stop = false;
        robot.driveTrain.startOdometry();
        while(opModeIsActive()) {
            if(gamepad1.right_bumper && !incrementUp) {
                incrementUp = true;
                increment *= 10;
            } else if (incrementUp && !gamepad1.right_bumper) {
                incrementUp = false;
            } else if(gamepad1.left_bumper && !incrementDown) {
                incrementDown = true;
                increment *= 0.1;
            } else if (incrementDown && !gamepad1.left_bumper) {
                incrementDown = false;
            }

            if(gamepad1.dpad_right && !next) {
                next = true;
                index += 1;
            } else if (next && !gamepad1.dpad_right) {
                next = false;
            } else if(gamepad1.dpad_left && !previous) {
                previous = true;
                index -= 1;
            } else if (previous && !gamepad1.dpad_left) {
                previous = false;
            } else if(gamepad1.dpad_up && !increase) {
                increase = true;
            } else if (increase && !gamepad1.dpad_up) {
                increase = false;
                if(index == 0) kP += increment;
                if(index == 1) kI += increment;
                if(index == 2) kD += increment;
                if(index == 3) kA += increment;
                if(index == 4) kV += increment;
                if(index == 5) power += increment;
            } else if(gamepad1.dpad_down && !decrease) {
                decrease = true;
                if(index == 0) kP -= increment;
                if(index == 1) kI -= increment;
                if(index == 2) kD -= increment;
                if(index == 3) kA -= increment;
                if(index == 4) kV -= increment;
                if(index == 5) power -= increment;
            } else if (decrease && !gamepad1.dpad_down) {
                decrease = false;
            }

            if(gamepad1.left_trigger > 0.5 && !stop) {
                stop = true;
                robot.driveTrain.stopController();
                Thread.sleep(20);
                robot.driveTrain.setPower(0);
            } else if (gamepad1.left_trigger < 0.5 && stop) {
                stop = false;
            }

            if(gamepad1.right_trigger > 0.5 && !trigger) {
                trigger = true;
                if(robot.driveTrain.controller != null) robot.driveTrain.stopController();
                Thread.sleep(20);
                robot.driveTrain.lineDrive(24, power, kP, kI, kD, kA, kV);
            } else if (gamepad1.right_trigger < 0.5 && trigger) {
                trigger = false;
            }


            telemetry.addLine("P: " +  String.format("%.6f", kP) + "  I: " + String.format("%.6f", kI) + "  D: " + String.format("%.6f", kD));
            //telemetry.addLine("A: " +  String.format("%.6f", kA) + "  V: " + String.format("%.6f", kV));
            telemetry.addLine("Power: " + String.format("%.6f", power));
            telemetry.addLine("Increment: " + Double.toString(increment) + " Index: " + Integer.toString(index));
            telemetry.addLine("X-Coord: " + String.format("%.3f", robot.driveTrain.getPosition().x) + "  Y-Coord: " +  String.format("%.3f", robot.driveTrain.getPosition().y));
            telemetry.addLine("φ: " + String.format("%.3f", robot.driveTrain.getPosition().phi));
            if(robot.driveTrain.controller != null) telemetry.addLine("e(t): " +  String.format("%.6f", ((ControllerPID) robot.driveTrain.controller).getError()));
            telemetry.update();
            Thread.sleep(50);
        }
    }

    @Override
    public void stop() {
        robot.stop();
        try {
            Thread.sleep(30);
        } catch (InterruptedException e) {

        }
        super.stop();
    }
}
