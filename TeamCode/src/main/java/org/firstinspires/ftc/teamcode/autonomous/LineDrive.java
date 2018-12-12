package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.constructs.Robot;
import org.firstinspires.ftc.teamcode.navigation.Position;
import org.firstinspires.ftc.teamcode.utilities.misc.LinearOpMode;
import org.firstinspires.ftc.teamcode.utilities.misc.StaticLog;

/**
 * Created by LeviG on 12/4/2018.
 */
@Autonomous(name = "Line Test")
public class LineDrive extends LinearOpMode {

    public volatile Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        StaticLog.clearLog();
        robot = new Robot(hardwareMap);
        robot.init();
        waitForStart();
        long startTime = System.currentTimeMillis();
        robot.driveTrain.startOdometry();
        boolean isLive = true;
        robot.driveTrain.degreeTurn(360,0.5);
        Thread.sleep(10000);

        robot.driveTrain.stop();
        //StaticLog.addLine("Controller Ticks Past: " + Integer.toString(robot.driveTrain.ticks));
        //StaticLog.addLine("Odometry Ticks Past: " + Integer.toString(robot.driveTrain.odometricTracker.getPositions().size()));
        //StaticLog.addLine("Elapsed Time: " + Long.toString(System.currentTimeMillis()-startTime));
        /**while(this.opModeIsActive()) {
            if(isLive) {
                Position pos = robot.driveTrain.getPosition();
                telemetry.addData("X-Coord: ", String.format("%.3f", pos.x));
                telemetry.addData("Y-Coord: ", String.format("%.3f", pos.y));
                telemetry.addData("φ-Coord: ", String.format("%.3f", pos.phi));
                telemetry.update();
            }
            Thread.sleep(200);
        }**/
    }

    @Override
    public void stop() {
        robot.stop();
        super.stop();
    }
}
