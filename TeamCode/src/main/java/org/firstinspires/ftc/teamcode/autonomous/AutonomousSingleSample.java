package org.firstinspires.ftc.teamcode.autonomous;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.constructs.Robot;
import org.firstinspires.ftc.teamcode.constructs.Slides;
import org.firstinspires.ftc.teamcode.navigation.Position;
import org.firstinspires.ftc.teamcode.utilities.misc.LinearOpMode;
import org.firstinspires.ftc.teamcode.utilities.misc.MathFTC;
import org.firstinspires.ftc.teamcode.utilities.misc.StaticLog;
import org.opencv.core.Rect;
import org.opencv.core.Size;

/**
 * Created by LeviG on 12/16/2018.
 */
@Autonomous(name = "Autonomous Single Sample")
public class AutonomousSingleSample extends LinearOpMode {

    MineralPosition mineralPosition = null;
    Robot robot;
    Thread odTel;
    Position pos;
    private GoldDetector detector;

    public static final boolean HANGS = true;

    public enum MineralPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    @Override
    public void runOpMode() throws InterruptedException{
        //Clears log of previous contents
        StaticLog.clearLog();
        //Instantiates and initiates hardware
        robot = new Robot(hardwareMap);
        robot.init();
        if(HANGS) robot.lift.setPower(0.38);
        //Waits for game start
        while (!isStarted()) {
            synchronized (this) {
                try {
                    telemetry.addLine("Waiting for start...");
                    telemetry.update();
                    Thread.sleep(1);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }
        }
        if(HANGS) {
            robot.slides.setArmPosition(Slides.Arm.IN);
            initiDetector();
            boolean found = false;
            Thread.sleep(300);
            robot.setDeposit(Robot.Deposit.HIGH);
            detector.enable();
            Thread.sleep(750);
            int i = 0;
            while (i < 5) {
                Rect foundRect = detector.getFoundRect();
                if(foundRect != null) {
                    if(foundRect.area() > 2) {
                        found = true;
                        mineralPosition = 0.5*(foundRect.tl().y+foundRect.br().y) < 240 ? MineralPosition.LEFT : MineralPosition.CENTER;
                        break;
                    }
                }
                i++;
                Thread.sleep(40);
            }
            detector.disable();
            if(!found) mineralPosition = MineralPosition.RIGHT;
            telemetry.addLine("Mineral Position: " + mineralPosition.name());
            telemetry.update();
            Thread.sleep(500);
            robot.lift.setPower(0.25);
            Thread.sleep(250);
            robot.lift.setPower(0.2);
            Thread.sleep(350);
            robot.lift.setPower(0.1);
            Thread.sleep(350);
            robot.lift.setPower(-0.2);
            Thread.sleep(750);
            robot.lift.setPower(0);
            Thread.sleep(750);
            //Boots odometry
            robot.driveTrain.startOdometry();
            odTel = new OdometryTel();
            odTel.start();
            Thread.sleep(200);
            //Moves hook off anchor point
            robot.lift.setPower(-0.2);
            Thread.sleep(200);
            robot.lift.setPower(0);
            //Slides hook out
            driveInches(2,0.35);
        }
        //Starts position tracking
        if(!HANGS) {
            robot.driveTrain.startOdometry();
            odTel = new OdometryTel();
            odTel.start();
            initiDetector();
            boolean found = false;
            Thread.sleep(100);
            detector.enable();
            Thread.sleep(750);
            int i = 0;
            while (i < 5) {
                Rect foundRect = detector.getFoundRect();
                if(foundRect != null) {
                    if(foundRect.area() > 2) {
                        found = true;
                        mineralPosition = 0.5*(foundRect.tl().y+foundRect.br().y) < 240 ? MineralPosition.LEFT : MineralPosition.CENTER;
                        break;
                    }
                }
                i++;
                Thread.sleep(40);
            }
            detector.disable();
            if(!found) mineralPosition = MineralPosition.RIGHT;
            telemetry.addLine("Mineral Position: " + mineralPosition.name());
            telemetry.update();
        }
        strafeInches(-14-robot.driveTrain.getPosition().y, 0.3);
        turnDegrees(0, 0.1);
        switch(mineralPosition) {
            case RIGHT:
                driveInches((-15-robot.driveTrain.getPosition().x), 0.32);
                break;
            case LEFT:
                driveInches((12-robot.driveTrain.getPosition().x), 0.32);
                break;
            case CENTER:
                driveInches(0-robot.driveTrain.getPosition().x, 0.25);
                break;
        }
        robot.driveTrain.setPower(-0.3, 0.3, 0.3, -0.3);
        Thread.sleep(1100);
        robot.driveTrain.setPower(0);
        Thread.sleep(150);
        strafeInches(-17-robot.driveTrain.getPosition().y, 0.3);
        turnDegrees(0, 0.1);
        //Moves forward
        driveInches(35-robot.driveTrain.getPosition().x,0.4);
        //Turns 45 degrees
        turnDegrees(-135,0.4, 0.15);
        robot.driveTrain.setPower(0.55, -0.55, -0.55, 0.55);
        Thread.sleep(1500);
        robot.driveTrain.setPower(0);
        Thread.sleep(150);
        strafeInches(-1, 0.28);
        pos = robot.driveTrain.getPosition();
        driveInches(-(83-pos.x*MathFTC.cos45+pos.y*MathFTC.sin45), 0.35, 0.1);
        robot.setDeposit(Robot.Deposit.DEPOSIT);
        robot.driveTrain.setPower(0);
        Thread.sleep(1600);
        robot.setDeposit(Robot.Deposit.MIDDLE);
        driveInches(62, 0.7);
        robot.driveTrain.setPower(-0.7);
        Thread.sleep(600);
        //**/
        robot.driveTrain.setPower(0);
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

    public void strafeInches(double distance, double p1) throws InterruptedException {
        Position startPos = robot.driveTrain.getPosition();
        double cos = Math.cos((Math.PI/180)*(startPos.phi+90));
        double sin = Math.sin((Math.PI/180)*(startPos.phi+90));
        double xF = startPos.x + distance*cos;
        double yF = startPos.y + distance*sin;
        if(Math.abs(MathFTC.distance(startPos.x, startPos.y, xF, yF, cos, sin)) < 0.5) return;
        double driveDirection = MathFTC.distance(startPos.x, startPos.y, xF, yF, cos, sin) > 0 ? 1 : -1;
        p1 *= driveDirection;
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
        robot.driveTrain.setPower(0);
        Thread.sleep(150);
    }

    public void turnDegrees(double phi, double p1) throws InterruptedException {
        if(Math.abs(robot.driveTrain.getPosition().phi - phi) < 8) return;
        double turnDirection = robot.driveTrain.getPosition().phi < phi ? 1 : -1;
        p1 *= turnDirection;
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
                    if(mineralPosition != null) telemetry.addLine("Mineral Position: " + mineralPosition.name());
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
