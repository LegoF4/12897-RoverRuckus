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
@Autonomous(name = "Autonomous New")
public class AutonomousDoubleSampleNew extends LinearOpMode {

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
            robot.slides.prepForEncoders(); // Locks slides in place
            robot.slides.setArmPosition(Slides.Arm.REST); //Rotates intake to REST position
            robot.setDeposit(Robot.Deposit.MIDDLE); //Sets deposition bucket to clear back cross-beam
            initiDetector();
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
            robot.driveTrain.startOdometry();
            odTel = new OdometryTel();
            odTel.start();
            Thread.sleep(200);
            //Moves hook off anchor point
            strafeInches(2.5, 0.35);
        }
        //Starts position tracking
        if(!HANGS) {
            //Initiates odometry at current position, along with telemetry feed
            robot.driveTrain.startOdometry();
            odTel = new OdometryTel();
            odTel.start();
            //Detect mineral and confirms sufficient size
            initiDetector();
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
        }

        Thread.sleep(300); //Pause after de-hang

        //Rotates slides out enough to drop intake
        int startPos = robot.slides.getPosition();
        int newPos = startPos - 750;
        robot.slides.setTargetPosition(newPos);
        while(robot.slides.getPosition() < newPos - 5 && opModeIsActive()) {
            Thread.sleep(20);
        }

        //Drops intake arms and turns intake on
        robot.slides.setArmPosition(Slides.Arm.OUT);
        robot.slides.setIntakeDirection(Slides.Intake.INTAKE);
        Thread.sleep(800);
        //Drives 3 inches forward so as to clear hook from latch
        robot.driveTrain.lineDrive(-5.5+robot.driveTrain.getPosition().x, 0.8);
        //Translates and orients robot for intake, then runs slides out appropriate amount
        switch (mineralPosition) {
            case CENTER:
                strafeInches(2-robot.driveTrain.getPosition().y, 0.32); //Strafes robot to center it
                robot.slides.setTargetPosition(startPos - 2420);
                while(robot.slides.getPosition() > (startPos - 2415) && opModeIsActive()) {
                    Thread.sleep(20);
                }
                break;
            case RIGHT:
                strafeInches(-1-robot.driveTrain.getPosition().y, 0.32); //Strafes robot to center it
                robot.driveTrain.degreeTurn(-31-robot.driveTrain.getPosition().phi,0.7);
                robot.slides.setTargetPosition(startPos - 2800);
                while(robot.slides.getPosition() > (startPos - 2795) && opModeIsActive()) {
                    Thread.sleep(20);
                }
                break;
            case LEFT:
                robot.driveTrain.degreeTurn(26-robot.driveTrain.getPosition().phi,0.7);
                robot.slides.setTargetPosition(startPos - 2800);
                while(robot.slides.getPosition() > (startPos - 2795) && opModeIsActive()) {
                    Thread.sleep(20);
                }
                break;
        }
        robot.slides.setTargetPosition(startPos);
        //Brings intake to rest position, leaving intake on
        robot.slides.setArmPosition(Slides.Arm.REST);
        Thread.sleep(800);
        robot.driveTrain.degreeTurn(0-robot.driveTrain.getPosition().phi,0.7);
        robot.driveTrain.lineDrive(-17.5-robot.driveTrain.getPosition().x, 0.8);
        strafeInches(21-robot.driveTrain.getPosition().y,0.7);
        robot.driveTrain.degreeTurn(-45-robot.driveTrain.getPosition().phi, 0.7);
        robot.driveTrain.setPower(0.6, -0.6, -0.6, 0.6);
        Thread.sleep(1000);
        robot.driveTrain.setPower(0);
        Thread.sleep(100);
        strafeInches(-1, 0.28);
        pos = robot.driveTrain.getPosition();
        robot.driveTrain.lineDrive(65-pos.x*MathFTC.cos45-pos.y*MathFTC.sin45, 0.8);
        strafeInches(-6,0.32);
        /**
        switch (mineralPosition) {
            case RIGHT:
                robot.driveTrain.degreeTurn(-83-robot.driveTrain.getPosition().phi, 0.6);
                break;
            case CENTER:
                robot.driveTrain.degreeTurn(-90-robot.driveTrain.getPosition().phi, 0.6);
                robot.slides.setTargetPosition(startPos - 2420);
                while(robot.slides.getPosition() > (startPos - 2415) && opModeIsActive()) {
                    Thread.sleep(20);
                }
                break;
            case LEFT:
                robot.driveTrain.degreeTurn(-125-robot.driveTrain.getPosition().phi, 0.6);
                break;
        }
        //robot.driveTrain.degreeTurn(-90-robot.driveTrain.getPosition().phi, 0.7);
        /**
        robot.setDeposit(Robot.Deposit.DEPOSIT);
        robot.driveTrain.setPower(0);
        Thread.sleep(1600);
        robot.setDeposit(Robot.Deposit.MIDDLE);
        //Drives to crater
        switch (mineralPosition) {
            case RIGHT:
                driveInches(35, 0.7);
                break;
            case CENTER:
                driveInches(51, 0.7);
                break;
            case LEFT:
                driveInches(62, 0.7);
                break;
        }
        robot.driveTrain.setPower(-0.7);
        Thread.sleep(600);
        robot.driveTrain.setPower(-0.3);
        while(opModeIsActive()) {
            Thread.sleep(10);
        }
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
        Thread.sleep(150);
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
