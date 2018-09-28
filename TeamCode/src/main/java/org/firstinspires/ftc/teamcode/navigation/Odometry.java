package org.firstinspires.ftc.teamcode.navigation;

import org.firstinspires.ftc.teamcode.utilities.hardware.Encoder;
import org.firstinspires.ftc.teamcode.utilities.misc.StaticLog;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by LeviG on 9/25/2018.
 */

public class Odometry {

    //Constants
    public static double wheelCircumference = 4 * Math.PI;; //In inches
    public static double robotDiameter = 9.2; //In inches

    public long freq; //In Hertz
    private volatile Encoder left;
    private volatile Encoder center;
    private volatile Encoder right;

    //Dynamic Tracking
    private volatile double xT = 0;
    private volatile double yT = 0;
    private volatile double phiT = 0;

    private volatile double x0;
    private volatile double y0;
    private volatile double phi0;

    //Thread Management
    private volatile Thread controlLoop;
    private volatile boolean isActive;

    public Odometry(Encoder left, Encoder center, Encoder right, long freq, double x, double y, double phi) {
        this.left = left;
        this.center = center;
        this.right = right;
        this.freq = freq;
        this.x0 = x;
        this.y0 = y;
        this.phi0 = phi;

        //Instantiates odometry thread
        synchronized (this) {
            this.controlLoop = new Odometry.OdometryThread();
            this.isActive = false;
        }
    }

    public Odometry(Encoder left, Encoder center, Encoder right, long freq) {
        this(left, center, right, freq, 0, 0, 0);
    }

    private class OdometryThread extends Thread {
        @Override
        public void run(){
            double leftPrior = wheelCircumference * left.getPosition();
            double centerPrior = wheelCircumference * center.getPosition();
            double rightPrior = wheelCircumference * right.getPosition();
            double leftCurrent;
            double centerCurrent;
            double rightCurrent;
            double phiCurrent;
            double deltaLeft;
            double deltaRight;
            double deltaCenter;
            double deltaPhi;
            double deltaLinear;
            double deltaX;
            double deltaY;
            while (isActive) {
                synchronized (this) {
                    leftCurrent = wheelCircumference * left.getPosition();
                    centerCurrent = wheelCircumference * center.getPosition();
                    rightCurrent = wheelCircumference * right.getPosition();
                    phiCurrent = phiT;
                }
                deltaLeft = leftCurrent - leftPrior;
                deltaCenter = centerCurrent - centerPrior;
                deltaRight = rightCurrent - rightPrior;

                deltaPhi = (180/Math.PI)*((deltaRight - deltaLeft) / robotDiameter);
                deltaLinear = Math.abs(deltaLeft) > Math.abs(deltaRight) ? deltaRight : deltaLeft;
                phiCurrent = (Math.PI/180)*(phiCurrent + deltaPhi*0.5);

                deltaX = deltaLinear*Math.cos(phiCurrent) + deltaCenter*Math.sin(phiCurrent);
                deltaY = deltaLinear*Math.sin(phiCurrent) + deltaCenter*Math.cos(phiCurrent);

                leftPrior = leftCurrent;
                centerPrior = centerCurrent;
                rightPrior = rightCurrent;
                synchronized (this) {
                    phiT += deltaPhi;
                    xT += deltaX;
                    yT += deltaY;
                }
            }
        }

        //Controls termination mechanic
        @Override
        public void interrupt() {
            StaticLog.addLine("Odometry Interrupted At: " + Long.toString(System.currentTimeMillis()));
            isActive = false;
            super.interrupt();
        }
    }

    synchronized public void init() {
        if(!isActive) {
            left.setZeroPosition();
            center.setZeroPosition();
            right.setZeroPosition();
        }
        StaticLog.addLine("Odometry Initiated At: " + Long.toString(System.currentTimeMillis()));
    }

    synchronized public List<Double> getPosition() {
        List<Double> coords = new ArrayList<Double>();
        coords.add(xT*Math.cos((Math.PI/180)*phi0)+yT*Math.sin((Math.PI/180)*phi0)+x0);
        coords.add(xT*Math.sin((Math.PI/180)*phi0)+yT*Math.cos((Math.PI/180)*phi0)+y0);
        coords.add(phiT+phi0);
        return coords;
    }

    /**
     * Starts a new instance of the odmometry loop
     */
    synchronized public void startControl() {
        StaticLog.addLine("Odometry Started At: " + Long.toString(System.currentTimeMillis()));
        isActive = true;
        controlLoop = new Odometry.OdometryThread();
        controlLoop.start();
    }

    /**
     * Terminates the odometry loop
     */
    synchronized public void stopControl() {
        isActive = false;
        controlLoop.interrupt();
    }
}
