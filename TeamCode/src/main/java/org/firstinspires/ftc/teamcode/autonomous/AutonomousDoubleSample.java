package org.firstinspires.ftc.teamcode.autonomous;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoTransitioner;
import org.firstinspires.ftc.teamcode.constructs.Robot;
import org.firstinspires.ftc.teamcode.navigation.Position;
import org.firstinspires.ftc.teamcode.utilities.misc.LinearOpMode;
import org.firstinspires.ftc.teamcode.utilities.misc.MathFTC;
import org.firstinspires.ftc.teamcode.utilities.misc.StaticLog;
import org.opencv.core.Rect;
import org.opencv.core.Size;

/**
 * Created by LeviG on 12/16/2018.
 */
@Autonomous(name = "Double Sample Autonomous")
public class AutonomousDoubleSample extends LinearOpMode {

    Robot robot;
    Thread odTel;
    private GoldDetector detector;

    @Override
    public void runOpMode() throws InterruptedException{
        //Clears log of previous contents
        StaticLog.clearLog();
        //Instantiates and initiates hardware
        robot = new Robot(hardwareMap);
        robot.init();
        //Waits for game start
        waitForStart();
        robot.driveTrain.startOdometry();
        odTel = new OdometryTel();
        odTel.start();
        //Insert de-hook code here
        //Straighten robot
        turnDegrees(0,0.2,0.1);
        //Back off
        driveInches((23.5-robot.driveTrain.getPosition().x), 0.2, 0.1);
        //Turn right
        turnDegrees(90,0.18,0.15);
        initiDetector();
        boolean found = false;
        Thread.sleep(100);
        detector.enable();
        Thread.sleep(500);
        int i = 0;
        while (i < 5) {
            Rect foundRect = detector.getFoundRect();
            if(foundRect != null) {
                if(foundRect.area() > 10) {
                    found = true;
                    break;
                }
            }
            i++;
            Thread.sleep(40);
        }
        detector.disable();
        telemetry.addLine("Found");
        telemetry.update();
        //Back off to 1st sample
        if(!found) {
            driveInches((-31+robot.driveTrain.getPosition().y), 0.2, 0.1);
            //Starts detector
            Thread.sleep(100);
            detector.enable();
            Thread.sleep(500);
            int j = 0;
            while (j < 5) {
                Rect foundRect = detector.getFoundRect();
                if(foundRect != null) {
                    if(foundRect.area() > 10) {
                        found = true;
                        break;
                    }
                }
                j++;
                Thread.sleep(40);
            }
            detector.disable();
        }
        if(!found) driveInches(30,0.2,0.1);
        strafeInches(-26.5+robot.driveTrain.getPosition().x, 0.3, 0.28);
        strafeInches(-16+robot.driveTrain.getPosition().x, 0.3, 0.28);
        turnDegrees(90, 0.1, 0.15);
        //Moves forward
        driveInches(40-robot.driveTrain.getPosition().y,0.2,0.1);
        //Turns 45 degrees
        turnDegrees(135,0.18,0.15);
        //Strafes into line with depot
        Position pos = robot.driveTrain.getPosition();
        strafeInches(-(43 -pos.x*MathFTC.cos45 -pos.y*MathFTC.sin45),0.3,0.28);
        turnDegrees(137,0.14,0.15);
        //Drives to depot
        pos = robot.driveTrain.getPosition();
        driveInches(65+pos.x*MathFTC.cos45-pos.y*MathFTC.sin45,0.2,0.1);
        //Deploy team-marker here

        //Drives to crater
        driveInches(40,0.6);
        robot.driveTrain.setPower(-0.35);
        Thread.sleep(350);
        //**/
        robot.driveTrain.setPower(0);
        Thread.sleep(200000);
    }

    @Override
    public void stop() {
        if(detector != null) detector.disable();
        if(robot != null) robot.stop();
        if (odTel != null) odTel.interrupt();
        super.stop();
    }

    public void driveInches(double distance, double p1) throws InterruptedException {
        Position startPos = robot.driveTrain.getPosition();
        double cos = Math.cos((Math.PI/180)*startPos.phi);
        double sin = Math.sin((Math.PI/180)*startPos.phi);
        double xF = startPos.x + distance*cos;
        double yF = startPos.y + distance*sin;
        if(Math.abs(MathFTC.distance(startPos.x, startPos.y, xF, yF, cos, sin)) < 0.5) return;
        double driveDirection = MathFTC.distance(startPos.x, startPos.y, xF, yF, cos, sin) > 0 ? 1 : -1;
        p1 *= driveDirection;
        robot.driveTrain.setPower(-p1);
        if(driveDirection > 0) {
            Position pos = robot.driveTrain.getPosition();
            while(!(MathFTC.distance(pos.x, pos.y, xF, yF, cos, sin) <= 0)) {
                Thread.sleep(50);
                pos = robot.driveTrain.getPosition();
            }
        } else {
            Position pos = robot.driveTrain.getPosition();
            while(!(MathFTC.distance(pos.x, pos.y, xF, yF, cos, sin) >= 0)) {
                Thread.sleep(50);
                pos = robot.driveTrain.getPosition();
            }
        }
        robot.driveTrain.setPower(0);
    }

    //Drives forward on current bearing until it has past a certain distance.
    public void driveInches(double distance, double p1, double p2) throws InterruptedException {
        Position startPos = robot.driveTrain.getPosition();
        double cos = Math.cos((Math.PI/180)*startPos.phi);
        double sin = Math.sin((Math.PI/180)*startPos.phi);
        double xF = startPos.x + distance*cos;
        double yF = startPos.y + distance*sin;
        if(Math.abs(MathFTC.distance(startPos.x, startPos.y, xF, yF, cos, sin)) < 0.5) return;
        double driveDirection = MathFTC.distance(startPos.x, startPos.y, xF, yF, cos, sin) > 0 ? 1 : -1;
        p1 *= driveDirection;
        p2 *= driveDirection;
        robot.driveTrain.setPower(-p1);
        if(driveDirection > 0) {
            Position pos = robot.driveTrain.getPosition();
            while(!(MathFTC.distance(pos.x, pos.y, xF, yF, cos, sin) <= 0)) {
                Thread.sleep(50);
                pos = robot.driveTrain.getPosition();
            }
        } else {
            Position pos = robot.driveTrain.getPosition();
            while(!(MathFTC.distance(pos.x, pos.y, xF, yF, cos, sin) >= 0)) {
                Thread.sleep(50);
                pos = robot.driveTrain.getPosition();
            }
        }
        robot.driveTrain.setPower(0);
        robot.driveTrain.setPower(p2);
        if(driveDirection > 0) {
            Position pos = robot.driveTrain.getPosition();
            while(!(MathFTC.distance(pos.x, pos.y, xF, yF, cos, sin) >= 0)) {
                Thread.sleep(50);
                pos = robot.driveTrain.getPosition();
            }
        } else {
            Position pos = robot.driveTrain.getPosition();
            while(!(MathFTC.distance(pos.x, pos.y, xF, yF, cos, sin) <= 0)) {
                Thread.sleep(50);
                pos = robot.driveTrain.getPosition();
            }
        }
        robot.driveTrain.setPower(0);
        Thread.sleep(150);
    }

    public void strafeInches(double distance, double p1, double p2) throws InterruptedException {
        Position startPos = robot.driveTrain.getPosition();
        double cos = Math.cos((Math.PI/180)*(startPos.phi+90));
        double sin = Math.sin((Math.PI/180)*(startPos.phi+90));
        double xF = startPos.x + distance*cos;
        double yF = startPos.y + distance*sin;
        if(Math.abs(MathFTC.distance(startPos.x, startPos.y, xF, yF, cos, sin)) < 0.5) return;
        double driveDirection = MathFTC.distance(startPos.x, startPos.y, xF, yF, cos, sin) > 0 ? 1 : -1;
        p1 *= driveDirection;
        p2 *= driveDirection;
        robot.driveTrain.setPower(p1, -p1, -p1, p1);
        if(driveDirection > 0) {
            Position pos = robot.driveTrain.getPosition();
            while(!(MathFTC.distance(pos.x, pos.y, xF, yF, cos, sin) <= 0)) {
                Thread.sleep(50);
                pos = robot.driveTrain.getPosition();
            }
        } else {
            Position pos = robot.driveTrain.getPosition();
            while(!(MathFTC.distance(pos.x, pos.y, xF, yF, cos, sin) >= 0)) {
                Thread.sleep(50);
                pos = robot.driveTrain.getPosition();
            }
        }
        Thread.sleep(50);
        robot.driveTrain.setPower(0);
        Thread.sleep(150);
        /**
         robot.driveTrain.setPower(-p2,p2,p2,-p2);
         if(driveDirection > 0) {
         Position pos = robot.driveTrain.getPosition();
         while(!(MathFTC.distance(pos.x, pos.y, xF, yF, cos, sin) >= 0)) {
         Thread.sleep(50);
         pos = robot.driveTrain.getPosition();
         }
         } else {
         Position pos = robot.driveTrain.getPosition();
         while(!(MathFTC.distance(pos.x, pos.y, xF, yF, cos, sin) <= 0)) {
         Thread.sleep(50);
         pos = robot.driveTrain.getPosition();
         }
         }
         robot.driveTrain.setPower(0);
         Thread.sleep(150);**/
    }

    public void turnDegrees(double phi, double p1, double p2) throws InterruptedException {
        if(Math.abs(robot.driveTrain.getPosition().phi - phi) < 8) return;
        double turnDirection = robot.driveTrain.getPosition().phi < phi ? 1 : -1;
        p1 *= turnDirection;
        p2 *= turnDirection;
        robot.driveTrain.setPower(p1, p1, -p1, -p1);
        if(turnDirection > 0) {
            while(!(robot.driveTrain.getPosition().phi >= phi)) {
                Thread.sleep(50);
            }
        } else {
            while (!(robot.driveTrain.getPosition().phi <= phi)) {
                Thread.sleep(50);
            }
        }
        /**
         Thread.sleep(50);
         robot.driveTrain.setPower(0);
         Thread.sleep(150);
         robot.driveTrain.setPower(-p2,-p2,p2,p2);
         if(turnDirection > 0) {
         while(!(robot.driveTrain.getPosition().phi <= phi)) {
         Thread.sleep(50);
         }
         } else {
         while (!(robot.driveTrain.getPosition().phi >= phi)) {
         Thread.sleep(50);
         }
         }
         //**/
        robot.driveTrain.setPower(0);
        Thread.sleep(150);

    }

    public void initiDetector() {
        detector = new GoldDetector();
        detector.setAdjustedSize(new Size(480, 270));
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();// Optional Tuning

        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.PERFECT_AREA; // Can also be PERFECT_AREA
        detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.perfectAreaScorer.weight = 0.05;
    }

    public class OdometryTel extends Thread {

        private volatile boolean isActive = false;
        @Override
        public void run() {
            isActive = true;
            while(isActive)    {
                synchronized (this) {
                    Position pos =  robot.driveTrain.getPosition();
                    telemetry.addData("X-Coord: ", String.format("%.3f", pos.x));
                    telemetry.addData("Y-Coord: ", String.format("%.3f", pos.y));
                    telemetry.addData("φ-Coord: ", String.format("%.3f", pos.phi));
                    telemetry.update();
                }

                try {
                    Thread.sleep(200);
                } catch (InterruptedException e) {
                    isActive = false;
                    this.interrupt();
                }
            }
        }

        @Override
        public void interrupt() {
            isActive = false;
            super.interrupt();
        }
    }
}
