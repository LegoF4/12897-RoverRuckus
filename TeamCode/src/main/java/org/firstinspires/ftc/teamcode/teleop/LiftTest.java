package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.utilities.misc.LinearOpMode;

@TeleOp(name="LiftTest")
public class LiftTest extends LinearOpMode {

    public DcMotor v1;
    public DcMotor v2;

    public void runOpMode() throws InterruptedException {

        v1 = hardwareMap.dcMotor.get("vl");
        v2 = hardwareMap.dcMotor.get("vr");
        v1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        v2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            double power;
            if(gamepad1.left_stick_y > 0.1) {
                power = Math.signum(gamepad1.left_stick_y)*Math.pow(gamepad1.left_stick_y,2);
            }
            else {
                power = 0.4*Math.signum(gamepad1.right_stick_y)*Math.pow(gamepad1.right_stick_y,2);
            }
            v1.setPower(power);
            v2.setPower(-power);
            telemetry.addLine("Power: " + Double.toString(power));
            telemetry.update();
            Thread.sleep(50);
        }
    }
}
