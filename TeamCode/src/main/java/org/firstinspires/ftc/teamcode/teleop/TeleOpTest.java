package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.constructs.Robot;
import org.firstinspires.ftc.teamcode.constructs.Slides;
import org.firstinspires.ftc.teamcode.utilities.gamepad.Button;
import org.firstinspires.ftc.teamcode.utilities.gamepad.ConditionedDigitalButton;
import org.firstinspires.ftc.teamcode.utilities.gamepad.DigitalButton;
import org.firstinspires.ftc.teamcode.utilities.gamepad.ToggleButton;
import org.firstinspires.ftc.teamcode.utilities.misc.LinearOpMode;
import org.firstinspires.ftc.teamcode.utilities.misc.StaticLog;
import org.firstinspires.ftc.teamcode.utilities.misc.TeleOpMode;

@TeleOp(name="TeleOpTest")
@Disabled
public class TeleOpTest extends LinearOpMode {

    public Robot robot;

    public volatile float powerScalar = 0.65f;

    public void runOpMode() throws InterruptedException {
        StaticLog.clearLog();
        robot = new Robot(hardwareMap);
        robot.init();
        waitForStart();
        boolean powerTogglePressed = false;
        //Loop variables
        while (opModeIsActive()) {
            //DRIVE CONTROLS
            if(gamepad1.right_stick_button && !powerTogglePressed) {
                powerScalar = powerScalar == 0.65f ? 0.35f : 0.65f;
                powerTogglePressed = true;
            } else if (powerTogglePressed && !gamepad1.right_stick_button) {
                powerTogglePressed = false;
            }
            setMotorPowers();

            //SLIDE CONTROLS
            if(gamepad1.dpad_up) {
                robot.slides.setPower(0.85);
            } else if (gamepad1.dpad_down) {
                robot.slides.setPower(-0.85);
            } else {
                robot.slides.setPower(0);
            }
            //INTAKE ARM CONTROLS

            //LIFT CONTROLS

            //DEPOSITION CONTROLS

            Thread.sleep(50);
        }
    }

    public void setMotorPowers() {
        float gamepad1LeftY = (float) (double) -gamepad1.left_stick_y;
        float gamepad1LeftX = (float) (double) gamepad1.left_stick_x;
        float gamepad1RightX = (float) (double) gamepad1.right_stick_x;

        float FrontLeft = -gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
        float FrontRight = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
        float BackRight = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
        float BackLeft = -gamepad1LeftY + gamepad1LeftX - gamepad1RightX;

        FrontRight = -scaleInput(FrontRight);
        FrontLeft = -scaleInput(FrontLeft);
        BackRight = -scaleInput(BackRight);
        BackLeft = -scaleInput(BackLeft);

        FrontRight = Range.clip(FrontRight, -1, 1);
        FrontLeft = Range.clip(FrontLeft, -1, 1);
        BackLeft = Range.clip(BackLeft, -1, 1);
        BackRight = Range.clip(BackRight, -1, 1);

        getRobot().driveTrain.setPower(-1*powerScalar*FrontLeft, -1*powerScalar*BackLeft, powerScalar*FrontRight, powerScalar*BackRight);

    }

    public synchronized Robot getRobot() {
        return this.robot;
    }

    public float scaleInput(float dVal) {
        float[] scaleArray = {0.0f, 0.05f, 0.09f, 0.10f, 0.12f, 0.15f, 0.18f, 0.24f,
                0.30f, 0.36f, 0.43f, 0.50f, 0.60f, 0.72f, 0.85f, 1.00f, 1.00f};

        // get the corresponding index for the scaleInput array.
        float index = (float) (dVal * 16.0);

        // index should be positive.
        if (index < 0f) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16f) {
            index = 16f;
        }

        // get value from the array.
        float dScale = 0.0f;
        if (dVal < 0f) {
            dScale = -scaleArray[(int) index];
        } else {
            dScale = scaleArray[(int) index];
        }

        // return scaled value.
        return dScale;
    }
}