package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constructs.Robot;
import org.firstinspires.ftc.teamcode.controllers.ControllerPID;
import org.firstinspires.ftc.teamcode.navigation.Position;
import org.firstinspires.ftc.teamcode.utilities.misc.LinearOpMode;
import org.firstinspires.ftc.teamcode.utilities.misc.StaticLog;

/**
 * Created by LeviG on 12/11/2018.
 */
@TeleOp(name = "PID Test", group = "Debug")
@Disabled
public class PIDTest extends LinearOpMode {

    Robot robot;
    Thread odTel;

    @Override
    public void runOpMode() throws InterruptedException{
        StaticLog.clearLog();
        robot = new Robot(hardwareMap);
        robot.init();
        waitForStart();

        double increment = 0.001;
        int index = 0;

        double kP = 0.032500;
        double kI = 0.000001;
        double kD = 0.000200;
        double kA = 0;
        double kV = 0;
        double power = 0.8;

        robot.driveTrain.startOdometry();
        odTel = new OdometryTel();
        odTel.start();
        Thread.sleep(100);
        robot.driveTrain.lineDrive(15.5, power, kP, kI, kD, kA, kV, true);
        robot.driveTrain.lineDrive(-15.5, power, kP, kI, kD, kA, kV, true);

        /**
        boolean stopped = false;
        long startTime = System.currentTimeMillis();
        while(opModeIsActive()) {
            if(System.currentTimeMillis() > startTime + 3000 && !stopped) {
                stopped = true;
                robot.driveTrain.stopController();
                Thread.sleep(100);
                robot.driveTrain.lineDrive(-24, power, kP, kI, kD, kA, kV, false);
            }
            telemetry.addLine("P: " +  String.format("%.6f", kP) + "  I: " + String.format("%.6f", kI) + "  D: " + String.format("%.6f", kD));
            //telemetry.addLine("A: " +  String.format("%.6f", kA) + "  V: " + String.format("%.6f", kV));
            telemetry.addLine("Power: " + String.format("%.6f", power));
            telemetry.addLine("Increment: " + Double.toString(increment) + " Index: " + Integer.toString(index));
            telemetry.addLine("X-Coord: " + String.format("%.3f", robot.driveTrain.getPosition().x) + "  Y-Coord: " +  String.format("%.3f", robot.driveTrain.getPosition().y));
            telemetry.addLine("φ: " + String.format("%.3f", robot.driveTrain.getPosition().phi));
            if(robot.driveTrain.controller != null) telemetry.addLine("e(t): " +  String.format("%.6f", ((ControllerPID) robot.driveTrain.controller).getError()));
            telemetry.update();
            Thread.sleep(50);
        }
         **/
    }

    @Override
    public void stop() {
        StaticLog.addLine("opMode.stop()");
        if (odTel != null) odTel.interrupt();
        if (robot != null) robot.stop();
        StaticLog.addLine("Stopped.");
        super.stop();
    }

    public class OdometryTel extends Thread {

        private volatile boolean isActiveO = false;

        @Override
        public void run() {
            isActiveO = true;
            while(isActiveO)    {
                synchronized (this) {
                    if(robot != null) {
                        Position pos =  robot.driveTrain.getPosition();
                        telemetry.addData("X-Coord: ", String.format("%.3f", pos.x));
                        telemetry.addData("Y-Coord: ", String.format("%.3f", pos.y));
                        telemetry.addData("φ-Coord: ", String.format("%.3f", pos.phi));
                    }
                    telemetry.update();
                }

                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    isActiveO = false;
                    this.interrupt();
                }
            }
        }

        @Override
        public void interrupt() {
            isActiveO = false;
            super.interrupt();
        }
    }
}
