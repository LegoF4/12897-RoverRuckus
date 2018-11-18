package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.utilities.misc.LinearOpMode;

@TeleOp(name="LiftTest")
public class LiftTest extends LinearOpMode {

    public DcMotor lift1;
    public DcMotor lift2;

    public void runOpMode() {

        lift1 = hardwareMap.dcMotor.get("l1");
        lift2 = hardwareMap.dcMotor.get("l2");

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                lift1.setPower(1);
                lift2.setPower(-1);
            } else if (!gamepad1.a) {
                lift1.setPower(0);
                lift2.setPower(0);
            }

            if (gamepad1.b) {
                lift1.setPower(-1);
                lift2.setPower(1);
            } else if (!gamepad1.b) {
                lift1.setPower(0);
                lift2.setPower(0);
            }

        }
    }
}
