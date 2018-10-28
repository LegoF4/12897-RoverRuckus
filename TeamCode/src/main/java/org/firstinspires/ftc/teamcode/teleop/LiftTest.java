package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.utilities.misc.LinearOpMode;

@TeleOp(name="LiftTest")
public class LiftTest extends LinearOpMode {

    public DcMotor lift;

    public void runOpMode() {

        lift = hardwareMap.dcMotor.get("l");

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                lift.setPower(1);
            } else if (!gamepad1.a) {
                lift.setPower(0);
            }

            if (gamepad1.b) {
                lift.setPower(-1);
            } else if (!gamepad1.b) {
                lift.setPower(0);
            }

        }
    }
}
