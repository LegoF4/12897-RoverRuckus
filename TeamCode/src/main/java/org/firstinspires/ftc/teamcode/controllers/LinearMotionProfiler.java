package org.firstinspires.ftc.teamcode.controllers;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by LeviG on 11/18/2018.
 */

public class LinearMotionProfiler implements FeedForward {

    public static final double distanceThreshold = 0.5; //Distance threshold, in inches
    private double kA = 1;
    private double kV = 1;

    private double frequency; //Hz
    private long T; //Milliseconds
    private double distance; //Inches
    private double accel; // Inches per second per second
    private double maxVel; // Inches per second

    private double[] path;
    private int size;
    private volatile int index; //Current index of robot on path

    /**
     *
     * @param frequency Hz
     * @param distance Positive, in inches
     * @param accel Inches per second per second
     * @param maxVel Inches per second
     */
    public LinearMotionProfiler(double frequency, double distance, double accel, double maxVel, double kA, double kV) {
        this.frequency = frequency;
        this.T = (long) (1000/frequency); //Converts Hz to ms
        this.distance = Math.abs(distance);
        this.accel = accel;
        this.maxVel = maxVel;
        this.index = 0;
        this.kA = kA;
        this.kV = kV;

        double criticalLength = 2*(maxVel*maxVel)/(accel);
        double tickAccel = (1/frequency)*accel;
        int criticalTicks = (int) (frequency*criticalLength);
        if(distance > distanceThreshold) {
            if(distance <= (criticalLength) ) {
                this.size = (int) Math.sqrt(2*distance/accel);
                path = new double[this.size];
                for (int i = 0; i <= this.size/2; i++) {
                    path[i] = kA*accel + kV*i*tickAccel;
                }
                for (int i = (int)this.size/2; i < 2*this.size/2; i++) {
                    path[i] = -kA*accel +kV*((this.size/2)*tickAccel - i*tickAccel);
                }
            } else {
                this.size = (int) (criticalTicks+((distance-criticalLength)/maxVel));
                path = new double[this.size];
                for (int i = 0; i <= criticalTicks/2; i++) {
                    path[i] = kA*accel + kV*i*tickAccel;
                }
                for (int i = 0; i <= criticalTicks/2; i++) {
                    path[i] = kV*maxVel;
                }
                for (int i = this.size - criticalTicks; i < this.size; i++) {
                    path[i] = -kA*accel +kV*((path.length/2)*tickAccel - i*tickAccel);
                }
            }
        }
        this.size = path.length;
    }

    @Override
    public double getNextTerm() {
        index += 1;
        if ((index+1)>size) {
            return 0;
        } else {
            return path[index];
        }
    }
}
