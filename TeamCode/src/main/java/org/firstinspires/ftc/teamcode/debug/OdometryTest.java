package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.navigation.Odometry;
import org.firstinspires.ftc.teamcode.navigation.Position;
import org.firstinspires.ftc.teamcode.utilities.hardware.Encoder;
import org.firstinspires.ftc.teamcode.utilities.hardware.EncoderMA3;
import org.firstinspires.ftc.teamcode.utilities.misc.LinearOpMode;
import org.firstinspires.ftc.teamcode.utilities.misc.StaticLog;

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
        odometricTracker.startTracking();
        StaticLog.clearLog();
        waitForStart();
        while(this.opModeIsActive()) {
            Position pos =  odometricTracker.getPosition();
            telemetry.addData("X-Coord: ", String.format("%.3f", pos.x));
            telemetry.addData("Y-Coord: ", String.format("%.3f", pos.y));
            telemetry.addData("φ-Coord: ", String.format("%.3f", pos.phi));
            telemetry.update();
            Thread.sleep(200);
        }
    }

    @Override
    public void stop() {
        odometricTracker.stopTracking();
        super.stop();
    }
}
