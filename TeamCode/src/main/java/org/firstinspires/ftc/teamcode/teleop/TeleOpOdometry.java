package org.firstinspires.ftc.teamcode.teleop;
import org.firstinspires.ftc.teamcode.utilities.misc.StaticLog;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utilities.misc.StaticLog;;
import org.firstinspires.ftc.teamcode.navigation.Odometry;
import org.firstinspires.ftc.teamcode.navigation.Position;
import org.firstinspires.ftc.teamcode.utilities.hardware.Encoder;
import org.firstinspires.ftc.teamcode.utilities.hardware.EncoderMA3;
import org.firstinspires.ftc.teamcode.utilities.misc.LinearOpMode;

@TeleOp(name="TeleOpMain")
public class TeleOpOdometry extends LinearOpMode {

    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor frontLeft;
    public DcMotor frontRight;

    private Odometry odometricTracker;

    private long startTime;

    public void runOpMode() throws InterruptedException{

        StaticLog.clearLog();;

        backLeft = hardwareMap.dcMotor.get("bl");
        backRight = hardwareMap.dcMotor.get("br");
        frontLeft = hardwareMap.dcMotor.get("fl");
        frontRight = hardwareMap.dcMotor.get("fr");
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Encoder left = new EncoderMA3(this.hardwareMap.analogInput.get("left"));
        Encoder center = new EncoderMA3(this.hardwareMap.analogInput.get("center"));
        Encoder right = new EncoderMA3(this.hardwareMap.analogInput.get("right"));
        odometricTracker = new Odometry(left, center, right, 25);
        Thread.sleep(1000);
        odometricTracker.init();
        Thread.sleep(1000);
        odometricTracker.startTracking();
        startTime = System.currentTimeMillis();

        waitForStart();

        while (opModeIsActive()) {
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

            frontRight.setPower(FrontRight);
            frontLeft.setPower(FrontLeft);
            backLeft.setPower(BackLeft);
            backRight.setPower(BackRight);

            Thread.sleep(50);
        }
    }

    @Override
    public void stop() {
        odometricTracker.stopTracking();
        StaticLog.addLine("Time Elapsed: " + Long.toString(System.currentTimeMillis()-startTime));
        StaticLog.addLine("Ticks: " + Integer.toString(odometricTracker.getPositions().size()));
        for(Position pos : odometricTracker.getPositions()) {
            StaticLog.addLine("X: " + Double.toString(pos.x) + ", Y: " + Double.toString(pos.y) + ", φ: " + Double.toString(pos.phi));
        }
        super.stop();
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
