package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utilities.misc.LinearOpMode;

import java.security.Policy;

@TeleOp(name="TeleOpOld")
@Disabled
public class TeleOpOld extends LinearOpMode {

    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor vl;
    public DcMotor vr;
    public DcMotor hl;
    public DcMotor hr;

    public CRServo intakeSpinLeft;
    public CRServo intakeSpinRight;
    public Servo intakeAngleLeft;
    public Servo intakeAngleRight;

    public Servo dumpAngleLeft;
    public Servo dumpAngleRight;


    public void runOpMode() throws InterruptedException {

        backLeft = hardwareMap.dcMotor.get("bl");
        backRight = hardwareMap.dcMotor.get("br");
        frontLeft = hardwareMap.dcMotor.get("fl");
        frontRight = hardwareMap.dcMotor.get("fr");
        vl = hardwareMap.get(DcMotor.class, "vl");
        vr = hardwareMap.get(DcMotor.class, "vr");
        hl = hardwareMap.get(DcMotor.class, "hl");
        hr = hardwareMap.get(DcMotor.class, "hr");

        intakeSpinLeft = hardwareMap.get(CRServo.class, "il");
        intakeSpinRight = hardwareMap.get(CRServo.class, "ir");
        intakeAngleLeft = hardwareMap.get(Servo.class, "al");
        intakeAngleRight = hardwareMap.get(Servo.class, "ar");

        dumpAngleLeft = hardwareMap.get(Servo.class, "dl");
        dumpAngleRight = hardwareMap.get(Servo.class, "dr");

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
        boolean toggle = false;
        boolean pressed = false;
        while (opModeIsActive()) {

            //REAL CODE
            float gamepad1LeftY = -gamepad1.left_stick_y;
            float gamepad1LeftX = gamepad1.left_stick_x;
            float gamepad1RightX = gamepad1.right_stick_x;

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


            double liftPower = gamepad1.right_trigger > 0.05 ? gamepad1.right_trigger : -1*gamepad1.left_trigger;
            liftPower = 0.35*Math.signum(liftPower)*Math.pow(liftPower,2);
            if (gamepad1.dpad_down) {
                vl.setPower(1);
                vr.setPower(-1);
            } else {
                vl.setPower(-liftPower);
                vr.setPower(liftPower);
            }

            if(gamepad1.a) {
                hl.setPower(0.5);
                hr.setPower(-0.5);
            } else if (gamepad1.y) {
                hl.setPower(-0.5);
                hr.setPower(0.5);
            } else {
                hl.setPower(0);
                hr.setPower(0);
            }

            //intake spin
            if (gamepad1.left_bumper) {
                intakeSpinLeft.setPower(1);
                intakeSpinRight.setPower(-1);
            } else if (gamepad1.right_bumper) {
                intakeSpinLeft.setPower(-1);
                intakeSpinRight.setPower(1);
            } //else {
                //il.setPower(0);
            //    ir.setPower(0);
            //}

//REAL CODE
            if(gamepad1.right_stick_button && !pressed) {
                toggle = !toggle;
                pressed = true;
            } else if (!gamepad1.right_stick_button && pressed) {
                pressed = false;
            }
            if(toggle) {
                intakeAngleLeft.setPosition(0.97);
                intakeAngleRight.setPosition(0.03);
            } else {
                intakeAngleLeft.setPosition(0.03);
                intakeAngleRight.setPosition(0.97);
            }
            if (gamepad1.x) {
                //intake down
                //dump middle
                dumpAngleRight.setPosition(0.85);
                dumpAngleLeft.setPosition(0.37);
            } else if (gamepad1.right_stick_button) {
                //dump down
                dumpAngleLeft.setPosition(0.24);
                dumpAngleRight.setPosition(0.97);
            } else if (gamepad1.b) {
                //dump up
                dumpAngleRight.setPosition(0.1);
                dumpAngleLeft.setPosition(0.97);
            }

            frontRight.setPower(0.55*FrontRight);
            frontLeft.setPower(0.55*FrontLeft);
            backLeft.setPower(0.55*BackLeft);
            backRight.setPower(0.55*BackRight);

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
            dScale = -scaleArray[(int) index];
        } else {
            dScale = scaleArray[(int) index];
        }

        // return scaled value.
        return dScale;
    }
}

