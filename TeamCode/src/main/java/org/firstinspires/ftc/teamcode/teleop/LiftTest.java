package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.utilities.misc.LinearOpMode;

@TeleOp(name="LiftTest")
public class LiftTest extends LinearOpMode {

    public DcMotor v1;
    public DcMotor v2;

    public void runOpMode() {

        v1 = hardwareMap.dcMotor.get("v1");
        v2 = hardwareMap.dcMotor.get("v2");
        v1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        v2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.left_stick_y > 0.1) {
                v1.setPower(Math.signum(gamepad1.left_stick_y)*Math.pow(gamepad1.left_stick_y,2));
                v2.setPower(-Math.signum(gamepad1.left_stick_y)*Math.pow(gamepad1.left_stick_y,2));
            }
            else {
                v1.setPower(0.5*Math.signum(gamepad1.right_stick_y)*Math.pow(gamepad1.right_stick_y,2));
                v2.setPower(.5*-Math.signum(gamepad1.right_stick_y)*Math.pow(gamepad1.right_stick_y,2));
            }
        }
    }
}
