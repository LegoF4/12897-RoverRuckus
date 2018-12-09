/*
 * Copyright <2017> <8148 Aleph Bots>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.teamcode.utilities.misc.StaticLog;

/**
 * Created by LeviG on 11/20/2017.
 */

public abstract class ControllerPID extends Controller {
    //Parameters
    public volatile double kP;
    public volatile double kD;
    public volatile double kI;
    public volatile FeedForward fF;
    public long T;
    public double powerThreshold;
    public double errorThreshold;
    //Desired Position

    public synchronized double getkP() {
        return kP;
    }

    public synchronized double getkD() {
        return kD;
    }

    public synchronized double getkI() {
        return kI;
    }

    public synchronized FeedForward getfF() {return fF; }

    public synchronized void setkP(double kP) {
        this.kP = kP;
    }

    public synchronized void setkD(double kD) {
        this.kD = kD;
    }

    public synchronized void setkI(double kI) {
        this.kI = kI;
    }

    public synchronized void setfF(FeedForward fF) {this.fF = fF; }

    public volatile double desiredPosition = 0;

    //Constructor
    public ControllerPID(double kP, double kD, double kI, double frequency, double powerThreshold, double errorThreshold) {
        super();
        //Sets parameters
        this.kP = kP;
        this.kD = kD;
        this.kI = kI;
        this.T = (long) (1000/frequency); //Converts Hz to ms
        this.powerThreshold = powerThreshold;
        this.errorThreshold = errorThreshold;
        this.desiredPosition = 0;
        this.fF = new FeedNull();
    }

    /**
     * Feed forward constructor
     */
    public ControllerPID(double kP, double kD, double kI, double frequency, double powerThreshold, double errorThreshold, FeedForward fF) {
        this(kP, kD, kI, frequency, powerThreshold, errorThreshold);
        this.fF = fF;
    }

    @Override
    protected void run() {
        double errorCurrent = getError();
        double errorPrevious = errorCurrent;
        double errorTotal = 0;
        double forwardTerm = 0;
        long startTime = System.currentTimeMillis();
        double timeLast = startTime;
        long sleepTime;
        while(isActive) {
            //Update error values
            errorPrevious = errorCurrent;
            errorCurrent = getError();
            forwardTerm = fF.getForwardTerm(System.currentTimeMillis()-startTime);
            StaticLog.addLine("Forward is: " + Double.toString(forwardTerm));
            if(Math.abs(errorCurrent) > errorThreshold) errorTotal += errorCurrent; //Includes powerThreshold to prevent long-term instability

            StaticLog.addLine("Error is: " + Double.toString(errorCurrent));
            StaticLog.addLine("Total Error is: " + Double.toString(errorTotal));
            StaticLog.addLine("Previous Error is: " + Double.toString(errorPrevious));//errorTotal = MathFTC.clamp(errorTotal, -1/kI, 1/kI);
            //Find desired power adjustment
            double u;
            u = kP*errorCurrent + kD*(errorCurrent-errorPrevious)/(T) + kI*errorTotal + forwardTerm;
            if(Math.abs(u) < powerThreshold) u = 0; //Prevents very low amplitude adjustments
            synchronized (this) {setOutput(u);}
            //Loop again in T ms
            try {
                sleepTime = (long) (T-(System.currentTimeMillis()-timeLast));
                if(sleepTime > 0) {
                    Thread.sleep(sleepTime);
                }
                //Terminate on exception
            } catch(InterruptedException e) {
                isActive = false;
                break;
            }
            timeLast = System.currentTimeMillis();
        }
    }

    /**
     *Get the current position of the system
     */
    public abstract double getCurrentPosition();

    /**
     * Set the desired position for the system
     * @param position Position intended
     */
    synchronized public void setDesiredPosition(double position) {
        this.desiredPosition = position;
    }

    /**
     * Obtains the current error of the system
     * @return Signed difference of desired position and current position
     */
    synchronized public double getError() {
        return this.desiredPosition - this.getCurrentPosition();
    }

}
