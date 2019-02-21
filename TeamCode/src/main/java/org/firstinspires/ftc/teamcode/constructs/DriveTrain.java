package org.firstinspires.ftc.teamcode.constructs;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.controllers.Controller;
import org.firstinspires.ftc.teamcode.controllers.ControllerPID;
import org.firstinspires.ftc.teamcode.controllers.FeedForward;
import org.firstinspires.ftc.teamcode.controllers.LinearMotionProfiler;
import org.firstinspires.ftc.teamcode.navigation.Odometry;
import org.firstinspires.ftc.teamcode.navigation.Position;
import org.firstinspires.ftc.teamcode.utilities.hardware.Encoder;
import org.firstinspires.ftc.teamcode.utilities.hardware.EncoderMA3;
import org.firstinspires.ftc.teamcode.utilities.misc.MathFTC;
import org.firstinspires.ftc.teamcode.utilities.misc.StaticLog;

/**
 * Created by LeviG on 11/16/2018.
 */

public class DriveTrain {

    public static final double acceleration = 12; // Measured in inches per second per second
    public static final double maxVelocity = 18; // Measured in inches per second
    public static final double alpha = 6; // Measured in degrees per second per second
    public static final double omega = 60; // Measured in degrees per second

    public static final double linearKP =  0.037500; //P term for linear driving PID controller
    public static final double linearKI =  0.000001; //I term for linear driving PID controller
    public static final double linearKD =  0.000200; //D term for linear driving PID controller
    public static final double linearKA =  0; //A term for linear driving FF controller
    public static final double linearKV =  0; //V term for linear driving FF controller

    public static final double angularKP = 0.019000; //P term for angular driving PID controller
    public static final double angularKI = 0.000025; //I term for angular driving PID controller
    public static final double angularKD = 0.000880; //D term for angular driving PID controller
    public static final double angularKA = 0; //A term for angular driving FF controller
    public static final double angularKV = 0; //V term for angular driving FF controller

    public volatile HardwareMap map;
    public volatile Odometry odometricTracker;
    public volatile Controller controller;

    private volatile DcMotor fr;
    private volatile DcMotor fl;
    private volatile DcMotor br;
    private volatile DcMotor bl;

    public volatile Encoder leftPod;
    public volatile Encoder centerPod;
    public volatile Encoder rightPod;

    private volatile long startTime;

    public DriveTrain(HardwareMap map, double x, double y, double phi) {
        this.map = map;
        fr = map.get(DcMotor.class,"fr");
        fl = map.get(DcMotor.class,"fl");
        br = map.get(DcMotor.class,"br");
        bl = map.get(DcMotor.class,"bl");

        leftPod = new EncoderMA3(map.get(AnalogInput.class,"left"));
        centerPod = new EncoderMA3(map.get(AnalogInput.class,"center"));
        rightPod = new EncoderMA3(map.get(AnalogInput.class,"right"));

        odometricTracker = new Odometry(leftPod, centerPod, rightPod, 100, x, y, phi);
        this.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public DriveTrain(HardwareMap map) {
        this(map, 0, 0, 0);
    }

    public synchronized void restartOdometry(double x, double y, double phi) throws InterruptedException {
        stopOdometry();
        odometricTracker = new Odometry(leftPod, centerPod, rightPod, 100, x, y, phi);
        startOdometry();
    }

    /**
     * Uses a FF controller to drive robot a fixed length on current bearing
     * @param distance Distance to be travelled, in inches
     * @param power Relative power to travel at
     */
    public void lineDrive(final double distance, double power) throws InterruptedException {
        lineDrive(distance, power, linearKP, linearKI, linearKD, linearKA, linearKV, true);
        setPower(0);
    }

    public void lineDrive(final double distance, double power, double kP, double kI, double kD, double kA, double kV, boolean termOnStop) throws InterruptedException {
        final double powerUsed = power;
        FeedForward feedForward = new LinearMotionProfiler(distance, acceleration, maxVelocity, kA, kV);
        final Position startPos = odometricTracker.getPosition();
        final double cos = Math.cos((Math.PI/180)*(startPos.phi));
        final double sin = Math.sin((Math.PI/180)*(startPos.phi));
        final double xF = startPos.x + distance*cos;
        final double yF = startPos.y + distance*sin;

        controller = new ControllerPID(kP, kD, kI, 100, 0.1, 0.25, feedForward, termOnStop, 1) {
            @Override
            public double getCurrentPosition() {
                //StaticLog.addLine("linearDrive.getCurrentPosition()");
                synchronized (this) {
                    if(DriveTrain.this != null) {
                        Position pos = odometricTracker.getPosition();
                        return -MathFTC.distance(pos.x, pos.y, xF, yF, cos, sin);
                    }
                    else return 0;
                }
            }

            @Override
            public void setOutput(double u) {
                //StaticLog.addLine("linearDrive.setOutput()");
                synchronized (this) {
                    if(DriveTrain.this != null) {
                        u = powerUsed*u;
                        setPower(u);
                    }
                }
            }
        };
        ((ControllerPID) controller).setDesiredPosition(0);
        controller.isActive = true;
        ((ControllerPID) controller).loop();
        //controller.startControl();
    }

    public void degreeTurn(double degree, double power) throws InterruptedException{
        degreeTurn(degree, power, angularKP, angularKI, angularKD, angularKA, angularKV, true);
    }

    /**
     * Uses a FF controller to turn robot a fixed distance on current location
     * @param degrees Degrees to be turned, positive is CCW
     * @param power Relative power to turn at
     */
    public void degreeTurn(double degrees, double power, double kP, double kI, double kD, double kA, double kV, boolean termOnStop) throws InterruptedException  {
        this.stopController();
        final double powerUsed = power;
        FeedForward feedForward = new LinearMotionProfiler(degrees, alpha, omega, kA, kV);
        StaticLog.addLine("Feed Forward initiated");
        final Position startPos = odometricTracker.getPosition();
        StaticLog.addLine("Initial position retrieved");
        controller = new ControllerPID(kP, kD, kI, 50, 0.1, 1.5, feedForward, termOnStop, 3) {
            @Override
            public double getCurrentPosition() {
                //StaticLog.addLine("degreeTurn.getCurrentPosition()");
                synchronized (this) {
                    return odometricTracker.getPosition().phi;
                }
            }

            @Override
            public void setOutput(double u) {
                //StaticLog.addLine("degreeTurn.setOutput()");
                synchronized (this) {
                    u = powerUsed*u;
                    setPower(u,u,-u,-u);
                }
            }
        };
        StaticLog.addLine("Controller initiated");
        ((ControllerPID) controller).setDesiredPosition(startPos.phi+degrees);
        controller.isActive = true;
        ((ControllerPID) controller).loop();
        //controller.startControl();
    }

    public synchronized void init() {

    }

    public synchronized boolean isDriving() {
        if(controller != null) return controller.isDone();
        else return false;
    }

    public void stop() {
        synchronized (this) {
            if(controller != null) controller.isActive = false;
        }
        //StaticLog.addLine("DriveTrain.stop()");
        //stopController();
        stopOdometry();
        setPower(0);
        bl.close();
        br.close();
        fr.close();
        fl.close();
        if(leftPod instanceof EncoderMA3) ((EncoderMA3) leftPod).close();
        if(centerPod instanceof EncoderMA3) ((EncoderMA3) centerPod).close();
        if(rightPod instanceof EncoderMA3) ((EncoderMA3) rightPod).close();
    }

    public synchronized void startOdometry() {
        startTime = System.currentTimeMillis();
        odometricTracker.startTracking();
    }

    public synchronized void stopOdometry() {
        //StaticLog.addLine("DriveTrain.stopOdometry()");
        if(odometricTracker != null) odometricTracker.stopTracking();
    }

    public synchronized void stopController() {
        //StaticLog.addLine("DriveTrain.stopController(). Controller is null: " + Boolean.toString(controller == null));
        if(controller != null) controller.stopControl();
    }

    public synchronized Position getPosition() {
        //StaticLog.addLine("DriveTrain.getPosition()");
        return odometricTracker.getPosition();
    }

    public synchronized void setPower(double power) {
        this.setPower(power, power, power, power);
    }

    public synchronized void setPower(double flP, double blP, double frP, double brP) {
        //StaticLog.addLine("DriveTrain.setPower()");
        flP = MathFTC.clamp(flP, -1, 1);
        frP = MathFTC.clamp(frP, -1, 1);
        blP = MathFTC.clamp(blP, -1, 1);
        brP = MathFTC.clamp(brP, -1, 1);
        fr.setPower(frP);
        br.setPower(brP);
        fl.setPower(-1*flP);
        bl.setPower(-1*blP);
    }

    public synchronized void setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior behaviour) {
        bl.setZeroPowerBehavior(behaviour);
        br.setZeroPowerBehavior(behaviour);
        fl.setZeroPowerBehavior(behaviour);
        fr.setZeroPowerBehavior(behaviour);
    }
}
