package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.constructs.Robot;
import org.firstinspires.ftc.teamcode.navigation.Position;
import org.firstinspires.ftc.teamcode.utilities.misc.LinearOpMode;
import org.firstinspires.ftc.teamcode.utilities.misc.MathFTC;
import org.firstinspires.ftc.teamcode.utilities.misc.StaticLog;

/**
 * Created by LeviG on 12/16/2018.
 */
@Autonomous(name = "Gold Autonomous")
public class AutonomousGold extends LinearOpMode {

    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException{
        StaticLog.clearLog();
        robot = new Robot(hardwareMap);
        robot.init();
        robot.lift.setPower(0.3);
        waitForStart();
        robot.lift.descend();
        Thread.sleep(4000);
        robot.lift.stopMoving();
        Thread.sleep(200);
        robot.driveTrain.startOdometry();
        //Insert de-hook code here

        //Straighten robot
        turnDegrees(0,0.3,0.15);
        //Back off
        driveInches((-18+robot.driveTrain.getPosition().x), 0.4, 0.15);
        //Turn right
        turnDegrees(-90,0.3,0.15);
        //Back off to 1st sample
        driveInches((-21+robot.driveTrain.getPosition().y), 0.4, 0.15);

    }

    @Override
    public void stop() {
        robot.stop();
        super.stop();
    }

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
        robot.driveTrain.setPower(p1);
        if(driveDirection > 0) {
            while(!(MathFTC.distance(startPos.x, startPos.y, xF, yF, cos, sin) <= 0)) {
                Thread.sleep(50);
            }
        } else {
            while(!(MathFTC.distance(startPos.x, startPos.y, xF, yF, cos, sin) >= 0)) {
                Thread.sleep(50);
            }
        }
        Thread.sleep(50);
        robot.driveTrain.setPower(0);
        Thread.sleep(150);
        robot.driveTrain.setPower(-p2);
        if(driveDirection > 0) {
            while(!(MathFTC.distance(startPos.x, startPos.y, xF, yF, cos, sin) >= 0)) {
                Thread.sleep(50);
            }
        } else {
            while(!(MathFTC.distance(startPos.x, startPos.y, xF, yF, cos, sin) <= 0)) {
                Thread.sleep(50);
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
        robot.driveTrain.setPower(-p1, p1, -p1, p1);
        if(driveDirection > 0) {
            while(!(MathFTC.distance(startPos.x, startPos.y, xF, yF, cos, sin) <= 0)) {
                Thread.sleep(50);
            }
        } else {
            while(!(MathFTC.distance(startPos.x, startPos.y, xF, yF, cos, sin) >= 0)) {
                Thread.sleep(50);
            }
        }
        Thread.sleep(50);
        robot.driveTrain.setPower(0);
        Thread.sleep(150);
        robot.driveTrain.setPower(p2,-p2,p2,-p2);
        if(driveDirection > 0) {
            while(!(MathFTC.distance(startPos.x, startPos.y, xF, yF, cos, sin) >= 0)) {
                Thread.sleep(50);
            }
        } else {
            while(!(MathFTC.distance(startPos.x, startPos.y, xF, yF, cos, sin) <= 0)) {
                Thread.sleep(50);
            }
        }
        robot.driveTrain.setPower(0);
        Thread.sleep(150);
    }

    public void turnDegrees(double phi, double p1, double p2) throws InterruptedException {
        if(Math.abs(robot.driveTrain.getPosition().phi - phi) < 2) return;
        double turnDirection = robot.driveTrain.getPosition().phi > phi ? 1 : -1;
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
}
