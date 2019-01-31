package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.utilities.misc.LinearOpMode;

/**
 * Created by LeviG on 1/4/2019.
 */
@TeleOp(name = "Intake Test")
@Disabled
public class IntakeTest extends LinearOpMode {

    CRServo ir;
    CRServo il;

    @Override
    public void runOpMode() throws InterruptedException {
        ir = hardwareMap.get(CRServo.class, "ir");
        il = hardwareMap.get(CRServo.class, "il");
        waitForStart();
        boolean pressed = false;
        while(opModeIsActive()) {
            if(gamepad1.a && !pressed) {
                pressed = true;
                ir.setPower(1);
                il.setPower(-1);
            } else if (gamepad1.b && !pressed) {
                pressed = true;
                ir.setPower(-1);
                il.setPower(1);
            } else if (!gamepad1.a && !gamepad1.b && pressed) {
                pressed = false;
            }
            Thread.sleep(50);
        }
    }

    @Override
    public void stop() {
        ir.setPower(0);
        il.setPower(0);
    }
}
