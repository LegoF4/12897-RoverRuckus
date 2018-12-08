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

    private double distance; //Inches
    private double accel; // Inches per second per second
    private double maxVel; // Inches per second

    private boolean isTriangular;
    private double totalTime;
    private double threshold1;
    private double threshold2;


    /**
     *
     * @param distance Positive, in inches
     * @param accel Inches per second per second
     * @param maxVel Inches per second
     */
    public LinearMotionProfiler(double distance, double accel, double maxVel, double kA, double kV) {
        this.distance = Math.abs(distance);
        this.accel = accel;
        this.maxVel = maxVel;
        this.kA = kA;
        this.kV = kV;
        this.isTriangular = true;
        totalTime = 0;
        if(this.distance > distanceThreshold) {
            double criticalLength = 2*(maxVel*maxVel)/(accel);
            if(this.distance > criticalLength) this.isTriangular = false;
            if(isTriangular) {
                threshold1 = 1000*Math.sqrt(2*this.distance/accel);
                totalTime = 2*threshold1;
                threshold2 = 0;
            } else {
                threshold1 = 1000*Math.sqrt(criticalLength/accel);
                threshold2 = threshold1 + 1000*(this.distance-criticalLength)/maxVel;
                totalTime = threshold2 + threshold1;
            }
        }
    }

    @Override
    public double getForwardTerm(double elapsedTime) {
        if(elapsedTime > totalTime || elapsedTime < 0) {
            return 0;
        } else if (isTriangular) {
            if(elapsedTime < threshold1) return kA*accel;
            else return -kA*accel;
        } else {
            if(elapsedTime < threshold1) return kA*accel;
            else if (elapsedTime > threshold1 && elapsedTime < threshold2) return kV*maxVel;
            else return -kA*accel;
        }
    }
}
