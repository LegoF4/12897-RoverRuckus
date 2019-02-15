package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.teamcode.utilities.misc.StaticLog;

/**
 * Created by LeviG on 11/19/2018.
 */

public abstract class Controller {


    //Thread Management
    protected volatile Thread controlLoop;
    protected volatile boolean isActive;

    public Controller() {
        //Instantiates control thread
        synchronized (this) {
            this.controlLoop = new Controller.ControlThread();
            this.isActive = false;
        }
    }

    /**
     * Actual looped thread, this is what runs constantly.
     */
    protected class ControlThread extends Thread {
        @Override
        public void run() {
            Controller.this.loop();
        }
        //Controls termination mechanic
        @Override
        public void interrupt() {
            StaticLog.addLine("Controller is " + (isActive ? "is active" : "is inactive") + ". Interrupted At: " + Long.toString(System.currentTimeMillis()));
            synchronized (this) {
                isActive = false;
            }
            super.interrupt();
        }
    }

    protected abstract void loop();

    /**
     * Starts a new instance of the control loop
     */
    synchronized public void startControl() {
        StaticLog.addLine("Controller start at: " + Long.toString(System.currentTimeMillis()));
        isActive = true;
        controlLoop = new Controller.ControlThread();
        controlLoop.start();
    }

    /**
     * Terminates the control loop
     */
    synchronized public void stopControl() {
        controlLoop.interrupt();
    }

    /**
     * Set the output of the system
     * @param u Output value from control equation
     */
    public abstract void setOutput(double u);

    public abstract boolean isDone();
}
