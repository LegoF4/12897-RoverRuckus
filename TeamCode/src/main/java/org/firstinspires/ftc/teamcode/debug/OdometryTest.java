package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.navigation.Odometry;
import org.firstinspires.ftc.teamcode.utilities.hardware.Encoder;
import org.firstinspires.ftc.teamcode.utilities.hardware.EncoderMA3;
import org.firstinspires.ftc.teamcode.utilities.misc.LinearOpMode;

/**
 * Created by LeviG on 9/27/2018.
 */
@TeleOp(name = "Odometry Test")
public class OdometryTest extends LinearOpMode {

    Odometry odometricTracker;
    @Override
    public void runOpMode() throws InterruptedException{
        Encoder left = new EncoderMA3(this.hardwareMap.analogInput.get("left"));
        Encoder center = new EncoderMA3(this.hardwareMap.analogInput.get("left"));
        Encoder right = new EncoderMA3(this.hardwareMap.analogInput.get("left"));
        odometricTracker = new Odometry(left, center, right, 50);
        odometricTracker.init();
        Thread.sleep(200);
        odometricTracker.startControl();
        while(this.opModeIsActive()) {

            Thread.sleep(100);
        }
    }

    @Override
    public void stop() {
        odometricTracker.stopControl();
        super.stop();
    }
}
