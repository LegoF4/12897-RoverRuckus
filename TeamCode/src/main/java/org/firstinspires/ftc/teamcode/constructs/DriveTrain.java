package org.firstinspires.ftc.teamcode.constructs;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.controllers.Controller;
import org.firstinspires.ftc.teamcode.controllers.ControllerMotionPlanning;
import org.firstinspires.ftc.teamcode.controllers.ControllerPID;
import org.firstinspires.ftc.teamcode.controllers.FeedForward;
import org.firstinspires.ftc.teamcode.controllers.LinearMotionProfiler;
import org.firstinspires.ftc.teamcode.navigation.Odometry;
import org.firstinspires.ftc.teamcode.utilities.hardware.Encoder;
import org.firstinspires.ftc.teamcode.utilities.hardware.EncoderMA3;

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

    public static final double angularKP = 0.1; //P term for angular driving PID controller
    public static final double angularKD = 0; //D term for angular driving PID controller
    public static final double angularKI = 0; //I term for angular driving PID controller
    public static final double angularKA = 0; //A term for angular driving FF controller
    public static final double angularKV = 0; //V term for angular driving FF controller

    public volatile HardwareMap map;
    private volatile Odometry odometricTracker;
    private volatile Controller controller;

    private volatile DcMotor rf;
    private volatile DcMotor lf;
    private volatile DcMotor rb;
    private volatile DcMotor lb;

    private volatile Encoder leftPod;
    private volatile Encoder centerPod;
    private volatile Encoder rightPod;

    private volatile boolean isDriving;

    public DriveTrain(HardwareMap map) {
        this.map = map;
        rf = map.get(DcMotor.class,"rf");
        lf = map.get(DcMotor.class,"lf");
        rb = map.get(DcMotor.class,"rb");
        lb = map.get(DcMotor.class,"lb");

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);

        leftPod = new EncoderMA3(map.get(AnalogInput.class,"leftPod"));
        centerPod = new EncoderMA3(map.get(AnalogInput.class,"centerPod"));
        rightPod = new EncoderMA3(map.get(AnalogInput.class,"rightPod"));

        odometricTracker = new Odometry(leftPod,centerPod,rightPod, 25);
        this.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);

        this.isDriving = false;
    }

    /**
     * Uses a FF controller to drive robot a fixed length on current bearing
     * @param distance Distance to be travelled, in inches
     * @param power Relative power to travel at
     */
    public void lineDrive(double distance, double power) throws InterruptedException {
        FeedForward feedForward = new LinearMotionProfiler(distance, acceleration*power, maxVelocity, linearKA, linearKV);
        controller = new ControllerPID(linearKP, linearKD, linearKI, 25, 0.02, 0.8, feedForward) {
            @Override
            public double getCurrentPosition() {
                return 0;
            }

            @Override
            public void setDesiredPosition(double position) {

            }

            @Override
            public void setOutput(double u) {
                setPower(u);
            }
        };
        controller.startControl();
    }

    /**
     * Uses a FF controller to turn robot a fixed distance on current location
     * @param degrees Degrees to be turned, positive is CCW
     * @param power Relative power to turn at
     */
    public void degreeTurn(double degrees, double power) throws InterruptedException  {
        LinearMotionProfiler feedForward = new LinearMotionProfiler(degrees, alpha*power, omega, angularKA, angularKV);
        controller = new ControllerMotionPlanning(feedForward, 25, 0.05) {
            @Override
            public void setOutput(double u) {
                setPower(u,u,-u,-u);
            }
        };
        controller.startControl();
    }

    public synchronized void init() {
        odometricTracker.init();
    }

    public synchronized void stop() {
        controller.stopControl();
        odometricTracker.stopTracking();
    }

    public synchronized void startOdometry() {
        odometricTracker.startTracking();
    }

    public synchronized void stopOdometry() {
        odometricTracker.stopTracking();
    }

    public synchronized void getPosition() {
        odometricTracker.getPosition();
    }

    public synchronized void setPower(double power) {
        this.setPower(power, power, power, power);
    }

    public synchronized void setPower(double lfP, double lbP, double rfP, double rbP) {
        rf.setPower(lfP);
        rb.setPower(rbP);
        lf.setPower(lfP);
        lb.setPower(lbP);
    }

    public synchronized void setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior behaviour) {
        lb.setZeroPowerBehavior(behaviour);
        rb.setZeroPowerBehavior(behaviour);
        lf.setZeroPowerBehavior(behaviour);
        rf.setZeroPowerBehavior(behaviour);
    }
}
