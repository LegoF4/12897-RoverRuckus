package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.navigation.Odometry;
import org.firstinspires.ftc.teamcode.utilities.hardware.Encoder;
import org.firstinspires.ftc.teamcode.utilities.hardware.EncoderMA3;
import org.firstinspires.ftc.teamcode.utilities.misc.LinearOpMode;
import org.firstinspires.ftc.teamcode.utilities.misc.StaticLog;

import java.util.List;

/**
 * Created by LeviG on 9/27/2018.
 */
@TeleOp(name = "Odometry Test")
public class OdometryTest extends LinearOpMode {

    private Odometry odometricTracker;

    @Override
    public void runOpMode() throws InterruptedException{
        Encoder left = new EncoderMA3(this.hardwareMap.analogInput.get("left"));
        Encoder center = new EncoderMA3(this.hardwareMap.analogInput.get("center"));
        Encoder right = new EncoderMA3(this.hardwareMap.analogInput.get("right"));
        odometricTracker = new Odometry(left, center, right, 25);
        Thread.sleep(1000);
        odometricTracker.init();
        Thread.sleep(1000);
        odometricTracker.startControl();
        StaticLog.clearLog();
        waitForStart();
        List<Double> coords;
        while(this.opModeIsActive()) {
            coords =  odometricTracker.getPosition();
            telemetry.addData("X-Coord: ", String.format("%.3f", coords.get(0)));
            telemetry.addData("Y-Coord: ", String.format("%.3f", coords.get(1)));
            telemetry.addData("φ-Coord: ", String.format("%.3f", coords.get(2)));
            telemetry.update();
            Thread.sleep(200);
        }
    }

    @Override
    public void stop() {
        odometricTracker.stopControl();
        super.stop();
    }
}
