package org.firstinspires.ftc.teamcode.debug;

import android.support.annotation.PluralsRes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constructs.Robot;
import org.firstinspires.ftc.teamcode.constructs.Slides;
import org.firstinspires.ftc.teamcode.utilities.misc.LinearOpMode;

/**
 * Created by LeviG on 1/5/2019.
 */
@TeleOp(name = "Servo Test", group="Debug")
@Disabled
public class ServoDebug extends LinearOpMode{

    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException{
        boolean pressed = false;
        boolean xPressed = false;
        boolean result = false;
        robot = new Robot(hardwareMap);
        robot.init();
        waitForStart();
        int state = 0;
        while(opModeIsActive()) {
            if(gamepad1.y && !pressed) {
                robot.slides.setArmPosition(Slides.Arm.OUT);
                state = 1;
                pressed = true;
            } else if (gamepad1.b && !gamepad1.y && !pressed) {
                robot.slides.setArmPosition(Slides.Arm.REST);
                state = 2;
                pressed = true;
            } else if(gamepad1.a && !gamepad1.y && !gamepad1.b && !pressed) {
                robot.slides.setArmPosition(Slides.Arm.IN);
                state = 3;
                pressed = true;
            } else if(!gamepad1.y && !gamepad1.b && !gamepad1.a && pressed){
                pressed = false;
            }
            telemetry.addLine("Pressed: " + Boolean.toString(pressed));
            telemetry.addLine("State : " + Integer.toString(state));
            if(gamepad1.x && !xPressed) {
                xPressed = true;
                result = !result;
            } else if (!gamepad1.x && xPressed) {
                xPressed = false;
            }
             //**/
            telemetry.addLine("X Result: " + Boolean.toString(result));
            telemetry.update();
            if(gamepad1.dpad_up) {
                robot.slides.setIntakePower(1);
            } else if (gamepad1.dpad_down) {
                robot.slides.setIntakeDirection(Slides.Intake.INTAKE);
            } else {
                robot.slides.setIntakeDirection(Slides.Intake.STOPPED);
            }
        }
        Thread.sleep(50);
    }

    @Override
    public void stop() {
        robot.stop();
        super.stop();
    }
}
