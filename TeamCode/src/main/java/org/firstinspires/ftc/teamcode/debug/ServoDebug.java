package org.firstinspires.ftc.teamcode.debug;

import android.support.annotation.PluralsRes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constructs.Robot;
import org.firstinspires.ftc.teamcode.utilities.misc.LinearOpMode;

/**
 * Created by LeviG on 1/5/2019.
 */
@TeleOp(name = "Servo Test")
public class ServoDebug extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException{
        boolean pressed = false;
        Robot robot = new Robot(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.a) {
                if(!pressed) robot.setDeposit(Robot.Deposit.MIDDLE);
            } else if (gamepad1.b) {
                if(!pressed) robot.setDeposit(Robot.Deposit.DEPOSIT);
            } else {
                if(pressed) pressed = true;
            }
        }
        Thread.sleep(50);
    }
}
