package org.firstinspires.ftc.teamcode.controllers;

/**
 * Created by LeviG on 11/19/2018.
 */

public abstract class ControllerMotionPlanning extends Controller {

    public long T;
    public double powerThreshold;
    public FeedForward fF;

    //Conclusion of current path
    private volatile boolean isDone;

    public ControllerMotionPlanning(FeedForward fF, double frequency, double powerThreshold) {
        this.T = (long) (1000/frequency); //Converts Hz to ms
        this.powerThreshold = powerThreshold;
        this.fF = fF;
        isDone = false;
    }

    @Override
    protected void loop() {
        long timeLast = System.currentTimeMillis();
        double power; /**
        while(isActive) {
            synchronized (this) {
                power = fF.getForwardTerm();
                setOutput(power);
            }
            if(power == 0) {
                this.isDone = true;
                break;
            }
            //Loop again in T ms
            try {
                Thread.sleep(T-(System.currentTimeMillis()-timeLast));
                //Terminate on exception
            } catch(InterruptedException e) {
                isActive = false;
                break;
            }
            timeLast = System.currentTimeMillis();
        }**/
    }

    public synchronized boolean isDone() {
        return isDone;
    }
}
