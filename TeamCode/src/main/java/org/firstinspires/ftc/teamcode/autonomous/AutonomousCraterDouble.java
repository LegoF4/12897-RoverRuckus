package org.firstinspires.ftc.teamcode.autonomous;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.constructs.Robot;
import org.firstinspires.ftc.teamcode.constructs.Slides;
import org.firstinspires.ftc.teamcode.navigation.Odometry;
import org.firstinspires.ftc.teamcode.navigation.Position;
import org.firstinspires.ftc.teamcode.utilities.misc.LinearOpMode;
import org.firstinspires.ftc.teamcode.utilities.misc.MathFTC;
import org.firstinspires.ftc.teamcode.utilities.misc.StaticLog;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Size;

/**
 * Created by LeviG on 12/16/2018.
 */
@Autonomous(name = "Autonomous Crater - DOUBLE")
public class AutonomousCraterDouble extends LinearOpMode {

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
        robot = new Robot(hardwareMap, 0, 0, 0);
        robot.init();
        if(HANGS) robot.lift.setPower(0.38);
        int startPos;
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
            robot.slides.prepForEncoders(); // Locks slides in place
            startPos = robot.slides.getPosition();
            robot.slides.setArmPosition(Slides.Arm.REST); //Rotates intake to REST position
            robot.setDeposit(Robot.Deposit.MIDDLE); //Sets deposition bucket to clear back cross-beam
            initDetector();
            boolean found = false;
            Thread.sleep(300);
            //Runs mineral detector algorithm
            detector.enable();
            Thread.sleep(750);
            int i = 0;
            while (i < 5) {
                Rect foundRect = detector.getFoundRect();
                if(foundRect != null) {
                    if(foundRect.area() > 2) {
                        found = true;
                        mineralPosition = 0.5*(foundRect.tl().y+foundRect.br().y) > 240 ? MineralPosition.LEFT : MineralPosition.CENTER;
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
            //Lowers robot and moves hook slightly off latch
            long liftStart = System.currentTimeMillis();
            robot.lift.setPower(-0.3);
            while(!robot.lift.isUp() && System.currentTimeMillis() < (liftStart + 4000) && opModeIsActive()) {
                telemetry.addLine("Lift Up: " + Boolean.toString(robot.lift.isUp()));
                telemetry.update();
                Thread.sleep(10);
            }
            robot.lift.setPower(0);
            //Boots odometry
            Thread.sleep(200);
            robot.driveTrain.startOdometry();
            //Moves hook off anchor point
            strafeInches(2, 0.35);
            Thread.sleep(350);
            robot.driveTrain.restartOdometry(0, 2.3, 0);
            odTel = new OdometryTel();
            odTel.start();
            Thread.sleep(200);
        }
        //Starts position tracking
        if(!HANGS) {
            //Initiates odometry at current position, along with telemetry feed
            robot.driveTrain.startOdometry();
            odTel = new OdometryTel();
            odTel.start();
            //Detect mineral and confirms sufficient size
            initDetector();
            boolean found = false;
            Thread.sleep(300);
            detector.enable();
            Thread.sleep(750);
            int i = 0;
            while (i < 5) {
                Rect foundRect = detector.getFoundRect();
                if(foundRect != null) {
                    if(foundRect.area() > 2) {
                        found = true;
                        mineralPosition = 0.5*(foundRect.tl().y+foundRect.br().y) > 240 ? MineralPosition.LEFT : MineralPosition.CENTER;
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
            startPos = robot.slides.getPosition();
        }
        //Rotates slides out enough to drop intake
        int newPos = startPos - 950;
        robot.slides.setTargetPosition(newPos);
        robot.slides.setPower(0.85);
        long slideTime = System.currentTimeMillis();
        while(robot.slides.getPosition() > newPos + 25 && opModeIsActive() && System.currentTimeMillis() < (slideTime + 2200) && opModeIsActive()) {
            Thread.sleep(20);
        }
        robot.slides.setPower(0);

        //Drops intake arms and turns intake on
        long extendTime = System.currentTimeMillis();
        robot.slides.setArmPosition(Slides.Arm.OUT);
        robot.slides.setIntakeDirection(Slides.Intake.INTAKE);

        //Lowers lift
        robot.lift.setPower(0.8);
        while(!robot.lift.isDown() && System.currentTimeMillis() < (extendTime + 900) && opModeIsActive()) {
            Thread.sleep(10);
        }
        robot.lift.setPower(0);

        //Drives forward so as to clear hook from latch
        robot.driveTrain.lineDrive(-5.5+robot.driveTrain.getPosition().x, 0.8);

        //Translates and orients robot for intake, then runs slides out appropriate amount
        switch (mineralPosition) {
            case CENTER:
                strafeInches(2-robot.driveTrain.getPosition().y, 0.32); //Strafes robot to center it
                //Run slides
                robot.slides.setTargetPosition(startPos - 2420);
                robot.slides.setPower(0.85);
                while(robot.slides.getPosition() > (startPos - 2405) && opModeIsActive()) {
                    Thread.sleep(20);
                }
                break;
            case RIGHT:
                robot.driveTrain.degreeTurn(-37-robot.driveTrain.getPosition().phi,0.7);
                //Run slides
                slideTime = System.currentTimeMillis();
                robot.slides.setTargetPosition(startPos - 2900);
                robot.slides.setPower(0.85);
                driveInches(-3, 0.2);
                //Wait for slides
                while(robot.slides.getPosition() > (startPos - 2885) && opModeIsActive() && System.currentTimeMillis() < (slideTime + 1200)) {
                    Thread.sleep(20);
                }
                driveInches(3, 0.2);
                break;
            case LEFT:
                robot.driveTrain.degreeTurn(26-robot.driveTrain.getPosition().phi,0.7);
                //Run slides
                robot.slides.setTargetPosition(startPos - 2800);
                robot.slides.setPower(0.85);
                slideTime = System.currentTimeMillis();
                while(robot.slides.getPosition() > (startPos - 2785) && opModeIsActive() && System.currentTimeMillis() < (slideTime + 1200)) {
                    Thread.sleep(20);
                }
                break;
        }
        //Brings slides in
        robot.slides.setIntakeDirection(Slides.Intake.INTAKE);
        robot.slides.setTargetPosition(startPos);

        //Brings intake to rest position, leaving intake on
        robot.slides.setArmPosition(Slides.Arm.REST);
        slideTime = System.currentTimeMillis();

        //Wait for slides to finish
        while(robot.slides.getPosition() < startPos - 20 && opModeIsActive() && System.currentTimeMillis() < (slideTime + 2200)) {
            Thread.sleep(20);
        }
        robot.slides.setIntakeDirection(Slides.Intake.STOPPED);

        //Re-zero, if necessary
        robot.driveTrain.degreeTurn(0-robot.driveTrain.getPosition().phi,0.7);
        robot.slides.setPower(0);
        //Drive to center between lander and samples
        robot.driveTrain.lineDrive(-17.5-robot.driveTrain.getPosition().x, 0.8);

        //Strafe out
        strafeInches(21-robot.driveTrain.getPosition().y,0.8);

        //Turn to be parallel to field walls
        robot.driveTrain.degreeTurn(-45-robot.driveTrain.getPosition().phi, 0.7);

        //Strafe into wall
        robot.driveTrain.setPower(0.6, -0.6, -0.6, 0.6);
        Thread.sleep(1000);
        robot.driveTrain.setPower(0);
        Thread.sleep(100);

        //Strafe 1 inch away from wall
        strafeInches(-1, 0.28);

        //Move to depot
        pos = robot.driveTrain.getPosition();
        driveInches(75-pos.x*MathFTC.cos45-pos.y*MathFTC.sin45, 0.4);

        //Move to rotate point for depot samples
        strafeInches(-7,0.32);

        //Rotate to face appropriate sample
        switch (mineralPosition) {
            case RIGHT:
                robot.driveTrain.degreeTurn(-72-robot.driveTrain.getPosition().phi, 0.7);
                break;
            case CENTER:
                robot.driveTrain.degreeTurn(-94-robot.driveTrain.getPosition().phi, 0.7);
                break;
            case LEFT:
                robot.driveTrain.degreeTurn(-115-robot.driveTrain.getPosition().phi, 0.7);
                break;
        }

        //Run slides
        newPos = startPos - 950;
        robot.slides.setTargetPosition(newPos);
        robot.slides.setPower(0.85);

        //Wait for slides to fully extend
        slideTime = System.currentTimeMillis();
        while(robot.slides.getPosition() > newPos + 10 && opModeIsActive() && System.currentTimeMillis() < (slideTime + 2200)) {
            Thread.sleep(20);
        }
        robot.slides.setPower(0);

        //Drops intake arms and turns intake on
        robot.slides.setArmPosition(Slides.Arm.OUT);
        robot.slides.setIntakeDirection(Slides.Intake.INTAKE);
        robot.setDeposit(Robot.Deposit.HIGH);
        Thread.sleep(600);
        slideTime = System.currentTimeMillis();
        robot.slides.setTargetPosition(startPos - 2800);
        robot.slides.setPower(0.85);
        while(robot.slides.getPosition() > (startPos - 2795) && opModeIsActive() && System.currentTimeMillis() < (slideTime + 1200)) {
            Thread.sleep(20);
        }
        robot.slides.setPower(0);

        //Bring deposit bucket, keep intake on
        robot.setDeposit(Robot.Deposit.MIDDLE);
        robot.slides.setIntakeDirection(Slides.Intake.INTAKE);
        robot.slides.setPower(0.82);
        robot.slides.setTargetPosition(startPos);

        //Brings slides in
        robot.slides.setArmPosition(Slides.Arm.REST);
        slideTime = System.currentTimeMillis();
        while(robot.slides.getPosition() < startPos - 20 && opModeIsActive() && System.currentTimeMillis() < (slideTime + 2200)) {
            Thread.sleep(20);
        }
        robot.slides.setIntakeDirection(Slides.Intake.STOPPED);

        //Turn to be parallel to wall again
        robot.driveTrain.degreeTurn(-38-robot.driveTrain.getPosition().phi, 0.7);

        //Brings intake in for transfer
        robot.slides.setArmPosition(Slides.Arm.IN);
        robot.slides.setIntakeDirection(Slides.Intake.INTAKE);
        Thread.sleep(500);

        //Strafe into wall
        robot.driveTrain.setPower(0.6, -0.6, -0.6, 0.6);
        Thread.sleep(800);
        robot.driveTrain.setPower(0);

        //Strafe out of wall
        strafeInches(-1, 0.28);
        robot.slides.setIntakeDirection(Slides.Intake.STOPPED);

        //Drive forward, halfway to crater
        driveInches(-30, 0.65);

        //Begin running the slides out
        robot.slides.setTargetPosition(startPos - 2800);
        robot.slides.setPower(0.9);

        //Move the rest of the way to the crater
        driveInches(-23, 0.65);
        //robot.driveTrain.setPower(0.3); //Run this if you need to stay parked in the crater
        robot.driveTrain.setPower(0);
         //**/
        Thread.sleep(200000);
    }

    @Override
    public void stop() {
        if (odTel != null) odTel.interrupt();
        if(detector != null) detector.disable();
        if(robot != null) robot.stop();
        super.stop();
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
        robot.driveTrain.setPower(0);
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
        robot.driveTrain.setPower(p1);
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
                Thread.sleep(20);
                pos = robot.driveTrain.getPosition();
            }
        } else {
            Position pos = robot.driveTrain.getPosition();
            while(!(MathFTC.distance(pos.x, pos.y, xF, yF, cos, sin) >= 0)) {
                Thread.sleep(20);
                pos = robot.driveTrain.getPosition();
            }
        }
        robot.driveTrain.setPower(0);
    }

    public void initDetector() {
        detector = new GoldDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();// Optional Tuning

        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.PERFECT_AREA; // Can also be PERFECT_AREA
        detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.perfectAreaScorer.weight = 0.05;
        detector.cropTLCorner = new Point(200, 15); //Sets the top left corner of the new image, in pixel (x,y) coordinates
        detector.cropBRCorner = new Point(500, 450); //Sets the bottom right corner of the new image, in pixel (x,y) coordinates

    }

    public class OdometryTel extends Thread {

        private volatile boolean isActive = false;
        @Override
        public void run() {
            isActive = true;
            while(isActive)    {
                synchronized (this) {
                    if(robot != null) {
                        Position pos =  robot.driveTrain.getPosition();
                        telemetry.addData("X-Coord: ", String.format("%.3f", pos.x));
                        telemetry.addData("Y-Coord: ", String.format("%.3f", pos.y));
                        telemetry.addData("φ-Coord: ", String.format("%.3f", pos.phi));
                    }
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
