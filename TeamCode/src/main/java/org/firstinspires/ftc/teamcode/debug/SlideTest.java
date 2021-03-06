package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.utilities.misc.LinearOpMode;

@TeleOp(name="Slide Test", group="Debug")
public class SlideTest extends LinearOpMode {

    public DcMotor v1;
    public DcMotor v2;
    public DigitalChannel channel;

    public void runOpMode() throws InterruptedException {

        v1 = hardwareMap.dcMotor.get("hl");
        v2 = hardwareMap.dcMotor.get("hr");
        channel = hardwareMap.get(DigitalChannel.class, "limit1");
        channel.setMode(DigitalChannel.Mode.INPUT);
        v1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        v2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            double power;
            if(Math.abs(gamepad1.left_stick_y) > 0.1) {
                power = 1*Math.signum(gamepad1.left_stick_y)*Math.pow(gamepad1.left_stick_y,2);
            }
            else {
                power = 0.75*Math.signum(gamepad1.right_stick_y)*Math.pow(gamepad1.right_stick_y,2);
            }
            v1.setPower(power);
            v2.setPower(-power);
            telemetry.addLine("Power: " + Double.toString(power));
            telemetry.addLine("Current State: " +(channel.getState() ? "NOT DETECTED" : "DETECTED"));
            telemetry.update();
            Thread.sleep(50);
        }
    }
}
