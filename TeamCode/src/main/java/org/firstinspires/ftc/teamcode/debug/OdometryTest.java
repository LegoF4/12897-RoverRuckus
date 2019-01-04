package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
public class OdometryTest extends LinearOpMode {

    private Odometry odometricTracker;

    private long startTime;

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
        startTime = System.currentTimeMillis();
        StaticLog.clearLog();
        waitForStart();
        boolean live = false;
        while(this.opModeIsActive()) {
            if(live) {
                Position pos =  odometricTracker.getPosition();
                telemetry.addData("X-Coord: ", String.format("%.3f", pos.x));
                telemetry.addData("Y-Coord: ", String.format("%.3f", pos.y));
                telemetry.addData("φ-Coord: ", String.format("%.3f", pos.phi));
                telemetry.update();
            }
            Thread.sleep(200);
        }
    }

    @Override
    public void stop() {
        odometricTracker.stopTracking();
        StaticLog.addLine("Time Elapsed: " + Long.toString(System.currentTimeMillis()-startTime));
        StaticLog.addLine("Ticks: " + Integer.toString(odometricTracker.getPositions().size()));
        for(Position pos : odometricTracker.getPositions()) {
            //StaticLog.addLine("X: " + Double.toString(pos.x) + ", Y: " + Double.toString(pos.y) + ", φ: " + Double.toString(pos.phi));
            StaticLog.addLine("X " + Double.toString(pos.x));
            StaticLog.addLine("Y " + Double.toString(pos.y));
            StaticLog.addLine("P " + Double.toString(pos.phi));
        }
        super.stop();
    }
}
