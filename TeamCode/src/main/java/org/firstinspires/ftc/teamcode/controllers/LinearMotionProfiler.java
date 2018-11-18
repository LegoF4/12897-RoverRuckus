package org.firstinspires.ftc.teamcode.controllers;

/**
 * Created by LeviG on 11/18/2018.
 */

public class LinearMotionProfiler implements FeedForward {

    private long frequency;
    private double distance;
    private double accel;
    private double maxVel;

    public LinearMotionProfiler(long frequency, double distance, double accel, double maxVel) {
        this.frequency = frequency;
        this.distance = distance;
        this.accel = accel;
        this.maxVel = maxVel;

    }

    @Override
    public double getFeedForward() {
        return 0;
    }
}
