package org.firstinspires.ftc.teamcode.constructs;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.controllers.ControllerPID;
import org.firstinspires.ftc.teamcode.controllers.FeedForward;
import org.firstinspires.ftc.teamcode.controllers.LinearMotionProfiler;
import org.firstinspires.ftc.teamcode.navigation.Odometry;
import org.firstinspires.ftc.teamcode.navigation.Position;
import org.firstinspires.ftc.teamcode.utilities.hardware.Encoder;
import org.firstinspires.ftc.teamcode.utilities.hardware.EncoderMA3;

/**
 * Created by LeviG on 11/16/2018.
 */

public class DriveTrain {

    public static final double acceleration = 6; // Measured in inches per second per second
    public static final double maxVelocity = 60; // Measured in inches per second


    public volatile HardwareMap map;
    private volatile Odometry odometricTracker;

    private volatile DcMotor rf;
    private volatile DcMotor lf;
    private volatile DcMotor rb;
    private volatile DcMotor lb;

    private volatile Encoder leftPod;
    private volatile Encoder centerPod;
    private volatile Encoder rightPod;


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
    }

    /**
     * Uses a PIDF controller to drive robot a fixed length on current bearing
     * @param distance Distance to be travelled, in inches
     * @param power Relative power to travel at
     */
    public void lineDrive(double distance, double power) throws InterruptedException {
        Position startPosition = odometricTracker.getPosition();
        LinearMotionProfiler feedForward = new LinearMotionProfiler(25, distance, acceleration*power, maxVelocity);
        ControllerPID controller = new ControllerPID(0.1,0,0,25, 0.05, 0.1, feedForward) {
            @Override
            public double getCurrentPosition() {
                Position pos = odometricTracker.getPosition();
                return 0;
            }

            @Override
            public void setDesiredPosition(double position) {
                this.desiredPosition = position;
            }

            @Override
            public void setOutput(double u) {
                setPower(u);
            }
        };
        controller.setDesiredPosition(distance);
        controller.startControl();
        Thread.sleep(((long) (distance*200)));
        controller.stopControl();
    }

    /**
     * Uses a PIDF controller to turn robot a fixed distance on current location
     * @param degrees Degrees to be turned, positive is CCW
     * @param power Relative power to turn at
     */
    public void degreeTurn(double degrees, double power) throws InterruptedException  {

    }

    public synchronized void init() {
        odometricTracker.init();
    }

    public synchronized void stop() {
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
