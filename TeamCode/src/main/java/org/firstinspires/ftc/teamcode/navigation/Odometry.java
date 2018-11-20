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

    private volatile long startTime;
    private volatile long count = 0;
    private volatile List<Position> positions;

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
        positions = new ArrayList<Position>();
        positions.add(new Position (x,y,phi,System.currentTimeMillis()));

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
        public void run() {
            double leftPrior = 0;
            double centerPrior = 0;
            double rightPrior = 0;
            boolean obtuse = false;
            if (obtuse)  StaticLog.addLine("-----Odometry Initiation Call-----");
            if (obtuse) StaticLog.addLine("Left Travel: " + Double.toString(leftPrior));
            if (obtuse) StaticLog.addLine("Center Travel: " + Double.toString(centerPrior));
            if (obtuse) StaticLog.addLine("Right Travel: " + Double.toString(rightPrior));
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
            startTime = System.currentTimeMillis();
            while (isActive) {
                if (obtuse) StaticLog.addLine("-----Odometry Encoder Call-----");
                synchronized (this) {
                    leftCurrent = wheelCircumference * left.getPosition() / 360;
                    centerCurrent = wheelCircumference * center.getPosition() / 360;
                    rightCurrent = wheelCircumference * right.getPosition() / 360;
                    phiCurrent = phiT;
                }
                if (obtuse) StaticLog.addLine("-----Odometry Cycle-----");
                if (obtuse) StaticLog.addLine("Left Travel: " + Double.toString(leftCurrent));
                if (obtuse) StaticLog.addLine("Center Travel: " + Double.toString(centerCurrent));
                if (obtuse) StaticLog.addLine("Right Travel: " + Double.toString(rightCurrent));

                deltaLeft = leftCurrent - leftPrior;
                deltaCenter = centerCurrent - centerPrior;
                deltaRight = -1*(rightCurrent - rightPrior);

                if (obtuse) StaticLog.addLine("Δ Left: " + Double.toString(deltaLeft));
                if (obtuse) StaticLog.addLine("Δ Center: " + Double.toString(deltaCenter));
                if (obtuse) StaticLog.addLine("Δ Right: " + Double.toString(deltaRight));

                deltaLinear = (deltaLeft + deltaRight) / 2;
                if (obtuse) StaticLog.addLine("Δ Linear: " + Double.toString(deltaLinear));

                deltaPhi = 360*((deltaRight - deltaLeft) / (2*robotDiameter*Math.PI));
                if (obtuse) StaticLog.addLine("Δφ: " + Double.toString(deltaPhi));
                phiCurrent = (Math.PI/180)*(phiCurrent + deltaPhi*0.5);
                if (obtuse) StaticLog.addLine("φ Current: " + Double.toString(phiCurrent));

                deltaX = deltaLinear*Math.cos(phiCurrent) + deltaCenter*Math.sin(phiCurrent);
                if (obtuse) StaticLog.addLine("ΔX: " + Double.toString(deltaX));
                deltaY = deltaLinear*Math.sin(phiCurrent) + deltaCenter*Math.cos(phiCurrent);
                if (obtuse) StaticLog.addLine("ΔY: " + Double.toString(deltaY));

                leftPrior = leftCurrent;
                centerPrior = centerCurrent;
                rightPrior = rightCurrent;
                synchronized (this) {
                    phiT += deltaPhi;
                    if (obtuse)StaticLog.addLine("φ(t): " + Double.toString(phiT));
                    xT += deltaX;
                    if (obtuse)StaticLog.addLine("x(t): " + Double.toString(xT));
                    yT += deltaY;
                    if (obtuse)StaticLog.addLine("y(t): " + Double.toString(yT));
                    positions.add(new Position(xT, yT, phiT, System.currentTimeMillis()));
                }

                try {
                    Thread.sleep(1000/freq);
                } catch (InterruptedException e) {
                    synchronized (this) {
                        isActive = false;
                    }
                }
                //**/
            }
            this.interrupt();
        }

        //Controls termination mechanic
        @Override
        public void interrupt() {
            StaticLog.addLine("Odometry Interrupted At: " + Long.toString(System.currentTimeMillis()));
            StaticLog.addLine("Counts: " + Long.toString(count));
            StaticLog.addLine("In: " + Long.toString(System.currentTimeMillis()-startTime));
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

    synchronized public Position getPosition() {
        double x = xT*Math.cos((Math.PI/180)*phi0)+yT*Math.sin((Math.PI/180)*phi0)+x0;
        double y = xT*Math.sin((Math.PI/180)*phi0)+yT*Math.cos((Math.PI/180)*phi0)+y0;
        double phi = phiT+phi0;
        return new Position(x,y,phi,System.currentTimeMillis());
    }

    /**
     * Starts a new instance of the odmometry loop
     */
    synchronized public void startTracking() {
        StaticLog.addLine("Odometry Started At: " + Long.toString(System.currentTimeMillis()));
        isActive = true;
        controlLoop = new Odometry.OdometryThread();
        controlLoop.start();
        //controlLoop.run();
    }

    /**
     * Terminates the odometry loop
     */
    synchronized public void stopTracking() {
        isActive = false;
        controlLoop.interrupt();
    }

    synchronized public List<Position> getPositions() {
        return this.positions;
    }
}
