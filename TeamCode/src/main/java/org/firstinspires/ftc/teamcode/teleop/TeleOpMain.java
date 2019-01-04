package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utilities.misc.LinearOpMode;

@TeleOp(name="TeleOpMain")
public class TeleOpMain extends LinearOpMode {

    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor vl;
    public DcMotor vr;
    public DcMotor hl;
    public DcMotor hr;


    public void runOpMode() throws InterruptedException{

        backLeft = hardwareMap.dcMotor.get("bl");
        backRight = hardwareMap.dcMotor.get("br");
        frontLeft = hardwareMap.dcMotor.get("fl");
        frontRight = hardwareMap.dcMotor.get("fr");
        vl = hardwareMap.get(DcMotor.class,"vl");
        vr = hardwareMap.get(DcMotor.class,"vr");
        hl = hardwareMap.get(DcMotor.class,"hl");
        hr = hardwareMap.get(DcMotor.class,"hr");
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        int count = 0;
        while (opModeIsActive()) {
            float gamepad1LeftY = -gamepad1.left_stick_y;
            float gamepad1LeftX = -gamepad1.left_stick_x;
            float gamepad1RightX = gamepad1.right_stick_x;

            double FrontLeft = -gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
            double FrontRight = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
            double BackRight = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
            double BackLeft = -gamepad1LeftY + gamepad1LeftX - gamepad1RightX;

            FrontRight = -scaleInput((float) FrontRight);
            FrontLeft = -scaleInput((float) FrontLeft);
            BackRight = -scaleInput((float) BackRight);
            BackLeft = -scaleInput((float) BackLeft);

            FrontRight = Range.clip(FrontRight, -1, 1);
            FrontLeft = Range.clip(FrontLeft, -1, 1);
            BackLeft = Range.clip(BackLeft, -1, 1);
            BackRight = Range.clip(BackRight, -1, 1);


            if(gamepad1.dpad_left) {
                FrontRight = -0.28;
                BackRight = 0.28;;
                FrontLeft = -0.28;
                BackLeft = 0.28;
            } else if (gamepad1.dpad_right) {
                FrontRight = 0.28;
                BackRight = -0.28;
                FrontLeft = 0.28;
                BackLeft = -0.28;
            } else if (gamepad1.dpad_up) {
                FrontRight = 0.28;
                BackRight = 0.28;
                FrontLeft = -0.28;
                BackLeft = -0.28;
            } else if (gamepad1.dpad_down) {
                FrontRight = -0.28;
                BackRight = -0.28;
                FrontLeft = 0.28;
                BackLeft = 0.28;
            }

            double liftPower = gamepad1.left_trigger > 0.05 ? gamepad1.left_trigger : -1*gamepad1.right_trigger;
            liftPower = 0.4*Math.signum(liftPower)*Math.pow(liftPower,2);
            vl.setPower(liftPower);
            vr.setPower(-liftPower);

            if(gamepad1.a) {
                hl.setPower(0.2);
                hr.setPower(-0.2);
            } else if (gamepad1.y) {
                hl.setPower(-0.2);
                hr.setPower(0.2);
            } else {
                hl.setPower(0);
                hr.setPower(0);
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
