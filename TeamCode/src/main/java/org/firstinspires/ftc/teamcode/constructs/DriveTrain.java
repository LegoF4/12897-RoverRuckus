package org.firstinspires.ftc.teamcode.constructs;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

    public static final double linearKP = 0.15; //P term for linear driving PID controller
    public static final double linearKD = 0; //D term for linear driving PID controller
    public static final double linearKI = 0; //I term for linear driving PID controller
    public static final double linearKA = 0.05; //A term for linear driving FF controller
    public static final double linearKV = 0.01; //V term for linear driving FF controller

    public static final double angularKP = 0.004; //P term for angular driving PID controller
    public static final double angularKD = 0; //D term for angular driving PID controller
    public static final double angularKI = 0; //I term for angular driving PID controller
    public static final double angularKA = 1; //A term for angular driving FF controller
    public static final double angularKV = 1; //V term for angular driving FF controller

    public volatile HardwareMap map;
    public volatile Odometry odometricTracker;
    private volatile Controller controller;

    private volatile DcMotor fr;
    private volatile DcMotor fl;
    private volatile DcMotor br;
    private volatile DcMotor bl;

    private volatile Encoder leftPod;
    private volatile Encoder centerPod;
    private volatile Encoder rightPod;

    private volatile boolean isDriving;

    private volatile long startTime;

    public DriveTrain(HardwareMap map) {
        this.map = map;
        fr = map.get(DcMotor.class,"fr");
        fl = map.get(DcMotor.class,"fl");
        br = map.get(DcMotor.class,"br");
        bl = map.get(DcMotor.class,"bl");

        leftPod = new EncoderMA3(map.get(AnalogInput.class,"left"));
        centerPod = new EncoderMA3(map.get(AnalogInput.class,"center"));
        rightPod = new EncoderMA3(map.get(AnalogInput.class,"right"));

        odometricTracker = new Odometry(leftPod,centerPod,rightPod, 100, 0, 4, 0);
        this.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);

        this.isDriving = false;
    }

    /**
     * Uses a FF controller to drive robot a fixed length on current bearing
     * @param distance Distance to be travelled, in inches
     * @param power Relative power to travel at
     */
    public void lineDrive(final double distance, double power) throws InterruptedException {
        final double powerUsed = power;
        FeedForward feedForward = new LinearMotionProfiler(distance, acceleration, maxVelocity, linearKA, linearKV);
        final Position startPos = odometricTracker.getPosition();
        final double cosPhi = Math.cos(((Math.PI)/(180))*startPos.phi);
        final double sinPhi = Math.sin(((Math.PI)/(180))*startPos.phi);
        final Position endPos = new Position(startPos.x+distance*cosPhi, startPos.y+distance*sinPhi, startPos.phi, System.currentTimeMillis());
        controller = new ControllerPID(linearKP, linearKD, linearKI, 100, 0.02, 0.5, feedForward) {
            @Override
            public double getCurrentPosition() {
                Position currentPos = odometricTracker.getPosition();
                double deltaX = endPos.x - currentPos.x;
                double deltaY = endPos.y - currentPos.y;
                double deltaU = deltaX*cosPhi + deltaY*sinPhi;
                double sign = deltaU > distance ? +1 : -1;
                return sign*Math.sqrt(deltaX*deltaX + deltaY*deltaY);
            }

            @Override
            public void setOutput(double u) {
                u = powerUsed*u;
                setPower(u);
            }
        };
        ((ControllerPID) controller).setDesiredPosition(distance);
        controller.startControl();
    }

    public volatile int ticks;

    public void degreeTurn(double degree, double power) throws InterruptedException{
        degreeTurn(degree, power, angularKP, angularKI, angularKD, angularKA, angularKV);
    }

    /**
     * Uses a FF controller to turn robot a fixed distance on current location
     * @param degrees Degrees to be turned, positive is CCW
     * @param power Relative power to turn at
     */
    public void degreeTurn(double degrees, double power, double kP, double kI, double kD, double kA, double kV) throws InterruptedException  {
        ticks = 0;
        final double powerUsed = power;
        FeedForward feedForward = new LinearMotionProfiler(degrees, alpha, omega, angularKA, angularKV);
        StaticLog.addLine("Feed Forward initiated");
        final Position startPos = odometricTracker.getPosition();
        StaticLog.addLine("Initial position retrieved");
        controller = new ControllerPID(kP, kD, kI, 100, 0.02, 1, feedForward) {
            @Override
            public double getCurrentPosition() {
                return getPosition().phi;
            }

            @Override
            public void setOutput(double u) {
                StaticLog.addLine("Output is: " + Double.toString(u));
                //ticks++;
                u = powerUsed*u;
                //setPower(u,u,-u,-u);
            }
        };
        StaticLog.addLine("Controller initiated");
        ((ControllerPID) controller).setDesiredPosition(startPos.phi+degrees);
        controller.startControl();
    }

    public synchronized void init() {
        odometricTracker.init();
    }

    public synchronized void stop() {
        if(controller != null) controller.stopControl();
        this.stopOdometry();
        setPower(0);
        bl.close();
        br.close();
        fr.close();
        fl.close();
    }

    public synchronized void startOdometry() {
        startTime = System.currentTimeMillis();
        odometricTracker.startTracking();
    }



    public synchronized void stopOdometry() {
        //StaticLog.addLine("Ticks Past: " + Integer.toString(odometricTracker.getPositions().size()));
        //StaticLog.addLine("Elapsed Time: " + Long.toString(System.currentTimeMillis()-startTime));
        odometricTracker.stopTracking();
    }

    public synchronized void stopController() {
        if(controller != null) controller.stopControl();
    }

    public synchronized Position getPosition() {
        return odometricTracker.getPosition();
    }

    public synchronized void setPower(double power) {
        this.setPower(power, power, power, power);
    }

    public synchronized void setPower(double flP, double blP, double frP, double brP) {
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
