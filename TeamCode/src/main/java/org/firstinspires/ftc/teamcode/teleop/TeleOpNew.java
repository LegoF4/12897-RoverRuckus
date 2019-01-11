package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utilities.misc.LinearOpMode;

@TeleOp(name="TeleOpNew")
public class TeleOpNew extends LinearOpMode {

    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor vl;
    public DcMotor vr;
    public DcMotor hl;
    public DcMotor hr;

    public CRServo il;
    public CRServo ir;
    public Servo al;
    public Servo ar;

    public Servo dl;
    public Servo dr;

    public enum Deposit {
        DEPOSIT,
        MIDDLE,
        DOWN
    }

    public enum Rollers {
        INTAKE,
        OUTPUT
    }

    public enum Arm {
        UP,
        DOWN
    }


    public void runOpMode() throws InterruptedException {

        backLeft = hardwareMap.dcMotor.get("bl");
        backRight = hardwareMap.dcMotor.get("br");
        frontLeft = hardwareMap.dcMotor.get("fl");
        frontRight = hardwareMap.dcMotor.get("fr");
        vl = hardwareMap.get(DcMotor.class, "vl");
        vr = hardwareMap.get(DcMotor.class, "vr");
        hl = hardwareMap.get(DcMotor.class, "hl");
        hr = hardwareMap.get(DcMotor.class, "hr");

        il = hardwareMap.get(CRServo.class, "il");
        ir = hardwareMap.get(CRServo.class, "ir");
        al = hardwareMap.get(Servo.class, "al");
        ar = hardwareMap.get(Servo.class, "ar");

        dl = hardwareMap.get(Servo.class, "dl");
        dr = hardwareMap.get(Servo.class, "dr");

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        //Loop variables
        boolean toggle = false;
        boolean pressed = false;
        float FrontLeft;
        float FrontRight;
        float BackRight;
        float BackLeft;
        float gamepad1LeftY;
        float gamepad1LeftX;
        float gamepad1RightX;
        double liftPower;
        Deposit deposit = null;
        Rollers rollers = null;
        Arm arm = null;

        while (opModeIsActive()) {
            //Motor Control
            gamepad1LeftY = -gamepad1.left_stick_y;
            gamepad1LeftX = gamepad1.left_stick_x;
            gamepad1RightX = gamepad1.right_stick_x;

            FrontLeft = -gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
            FrontRight = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
            BackRight = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
            BackLeft = -gamepad1LeftY + gamepad1LeftX - gamepad1RightX;

            FrontRight = -scaleInput(FrontRight);
            FrontLeft = -scaleInput(FrontLeft);
            BackRight = -scaleInput(BackRight);
            BackLeft = -scaleInput(BackLeft);

            FrontRight = Range.clip(FrontRight, -1, 1);
            FrontLeft = Range.clip(FrontLeft, -1, 1);
            BackLeft = Range.clip(BackLeft, -1, 1);
            BackRight = Range.clip(BackRight, -1, 1);

            //Lift Power
            liftPower = gamepad1.right_trigger > 0.05 ? gamepad1.right_trigger : -1*gamepad1.left_trigger;
            liftPower = 0.35*Math.signum(liftPower)*Math.pow(liftPower,2);
            if (gamepad1.dpad_down) {
                vl.setPower(1);
                vr.setPower(-1);
            } else {
                vl.setPower(-liftPower);
                vr.setPower(liftPower);
            }

            //Deposit Control
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

            //Intake Control
            if (gamepad1.left_bumper) {
                il.setPower(1);
                ir.setPower(-1);
            } else if (gamepad1.right_bumper) {
                il.setPower(-1);
                ir.setPower(1);
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
                al.setPosition(0.97);
                ar.setPosition(0.03);
            } else {
                al.setPosition(0.03);
                ar.setPosition(0.97);
            }
            if (gamepad1.x) {
                //intake down
                //dump middle
                dr.setPosition(0.85);
                dl.setPosition(0.37);
            } else if (gamepad1.right_stick_button) {
                //dump down
                dl.setPosition(0.24);
                dr.setPosition(0.97);
            } else if (gamepad1.b) {
                //dump up
                dr.setPosition(0.1);
                dl.setPosition(0.97);
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

