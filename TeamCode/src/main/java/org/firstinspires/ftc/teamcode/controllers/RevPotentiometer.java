/*
 * Copyright <2017> <8148 Aleph Bots>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.library.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;

/**
 * Created by LeviG on 11/12/2017.
 */

public class RevPotentiometer{

    public double Vin;
    public double Rt;

    private static double minPos = 0;
    private static double maxPos = 270;

    volatile AnalogInput input;

    public RevPotentiometer(AnalogInput input) {
        this(input, 3.3, 10000);
    }

    public RevPotentiometer(AnalogInput input, double Vin, double Rt) {
        this.input = input;
        this.Rt = Rt;
        this.Vin = Vin;
    }

    /**
     * Returns the current angle of the potentiometer as a double from 0 to 270 degrees.
     * @return
     */
    synchronized public double getAngle() {
        double voltage = input.getVoltage();
        if (voltage < 0.005) return this.getMaxPos();
        return (135*(5000*Vin + Rt*voltage - Math.sqrt(25000000*Vin*Vin - 10000*Vin*Rt*voltage + Rt*(20000 + Rt)*voltage*voltage)))/(Rt*voltage);
    }

    /**
     * Returns the output voltage of the sensor for a given angle
     * @param angle Am angle, measured counter-clockwise, in degrees from 0 to 270.
     * @return A voltage from 0 to 3.34.
     */
    synchronized public double getVoltage(double angle) {
        return (1350000*(270-angle)*Vin) / (- Rt * Math.pow(angle, 2) + 270*Rt*angle + 364500000);
    }

    public double getMinPos() {return minPos;}

    public double getMaxPos() {return maxPos;}



}
