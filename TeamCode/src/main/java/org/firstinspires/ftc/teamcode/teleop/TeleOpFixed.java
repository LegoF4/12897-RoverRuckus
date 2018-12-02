package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utilities.misc.LinearOpMode;

@TeleOp(name="TeleOFix")
public class TeleOpFixed extends LinearOpMode {

    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor frontLeft;
    public DcMotor frontRight;


    public void runOpMode() throws InterruptedException{

        backLeft = hardwareMap.dcMotor.get("bl");
        backRight = hardwareMap.dcMotor.get("br");
        frontLeft = hardwareMap.dcMotor.get("fl");
        frontRight = hardwareMap.dcMotor.get("fr");
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {

            float FrontLeft;
            float FrontRight;
            float BackRight;
            float BackLeft;

            if(gamepad1.dpad_left) {
                FrontRight = -1;
                BackRight = 1;
                FrontLeft = -1;
                BackLeft = 1;
            } else if (gamepad1.dpad_right) {
                FrontRight = 1;
                BackRight = -1;
                FrontLeft = 1;
                BackLeft = -1;
            } else if (gamepad1.dpad_up) {
                FrontRight = 1;
                BackRight = 1;
                FrontLeft = -1;
                BackLeft = -1;
            } else if (gamepad1.dpad_down) {
                FrontRight = -1;
                BackRight = -1;
                FrontLeft = 1;
                BackLeft = 1;
            } else {
                FrontRight = 0;
                BackRight = 0;
                FrontLeft = 0;
                BackLeft = 0;
            }

            frontRight.setPower(FrontRight);
            frontLeft.setPower(FrontLeft);
            backLeft.setPower(BackLeft);
            backRight.setPower(BackRight);

            telemetry.addLine("Front Right: " + Double.toString(FrontRight));
            telemetry.addLine("Front Left: " + Double.toString(FrontLeft));
            telemetry.addLine("Back Right: " + Double.toString(BackRight));
            telemetry.addLine("Back Left: " + Double.toString(BackLeft));
            telemetry.update();
            Thread.sleep(50);
        }
    }

    float scaleInput(float dVal) {
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
            dScale = -scaleArray[(int)index];
        } else {
            dScale = scaleArray[(int)index];
        }

        // return scaled value.
        return dScale;
    }

}
