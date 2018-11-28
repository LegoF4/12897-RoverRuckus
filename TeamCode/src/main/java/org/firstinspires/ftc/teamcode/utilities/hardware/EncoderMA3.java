package org.firstinspires.ftc.teamcode.utilities.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utilities.misc.StaticLog;
import org.firstinspires.ftc.teamcode.utilities.misc.MathFTC;

/**
 * Created by LeviG on 9/25/2018.
 */

public class EncoderMA3 extends Encoder {

    //Hardware Parameters
    public static double MaxVoltage = 3.26; //In volts
    public static double MinVoltage = 0.0; //In volts
    public static double threshold = 0.4; //In degrees

    public volatile AnalogInput encoder; //The hardware device

    //Tracking Variables
    private volatile double zeroPosition; //In absolute degrees
    private volatile double priorPosition; //In relative degrees
    private volatile double currentPosition; //In relative degrees
    private volatile double firstDerivative; //In degrees per dynamic tick
    private volatile double measuredPosition; //In absolute degrees


    public EncoderMA3(AnalogInput input) {
        this.encoder = input;
        input.resetDeviceConfigurationForOpMode();
        this.zeroPosition = 360*this.encoder.getVoltage()/(MaxVoltage - MinVoltage); //In degrees
        this.currentPosition = this.zeroPosition;
        this.priorPosition = this.currentPosition;
        this.firstDerivative = 0;
    }

    /**
     * Retrieves the encoders current position
     * @return The number of degrees traversed from the zero position
     */
    public synchronized double getPosition() {
        measuredPosition = 360*encoder.getVoltage()/(MaxVoltage - MinVoltage);

        if(firstDerivative >= 0 && measuredPosition < priorPosition && Math.abs(measuredPosition - priorPosition) > 90) {
            firstDerivative = measuredPosition + 360 - priorPosition; //Positive wrap-around
        } else if (firstDerivative <= 0 && measuredPosition > priorPosition && Math.abs(measuredPosition - priorPosition) > 90) {
            firstDerivative = -1*(priorPosition + 360 - measuredPosition); //Negative wrap-around
        } else if(MathFTC.inRange(Math.abs(measuredPosition - priorPosition), threshold, 180)) {
            firstDerivative = measuredPosition - priorPosition; //Normal motion
        } else {
            firstDerivative = 0; //Holding still
        }

        if(Math.abs(firstDerivative) > 180) {
            firstDerivative = 0; //Accounts for large, single-tick fluctuations in encoder value
        }
        if(firstDerivative != 0) { //If in valid motion, update prior position
            priorPosition = measuredPosition;
        }

        currentPosition += firstDerivative; //Adjust dynamic motion track
        return (currentPosition - zeroPosition); //Corrects for zero position and over-counting
    }

    /**
     * Retrieves the encoders current position
     * @return The number of degrees traversed from the zero position
     */
    public synchronized double getPosition(double measuredPosition) {
        if(firstDerivative >= 0 && measuredPosition < priorPosition && Math.abs(measuredPosition - priorPosition) > 90) {
            firstDerivative = measuredPosition + 360 - priorPosition; //Positive wrap-around
        } else if (firstDerivative <= 0 && measuredPosition > priorPosition && Math.abs(measuredPosition - priorPosition) > 90) {
            firstDerivative = -1*(priorPosition + 360 - measuredPosition); //Negative wrap-around
        } else if(MathFTC.inRange(Math.abs(measuredPosition - priorPosition), threshold, 180)) {
            firstDerivative = measuredPosition - priorPosition; //Normal motion
        } else {
            firstDerivative = 0; //Holding still
        }

        if(Math.abs(firstDerivative) > 180) {
            firstDerivative = 0; //Accounts for large, single-tick fluctuations in encoder value
        }
        if(firstDerivative != 0) { //If in valid motion, update prior position
            priorPosition = measuredPosition;
        }

        currentPosition += firstDerivative; //Adjust dynamic motion track
        return (currentPosition - zeroPosition); //Corrects for zero position and over-counting
    }

    /**
     * Sets encoder zero position to current position.
     */
    public synchronized void setZeroPosition() {
        this.zeroPosition = 360*encoder.getVoltage()/(MaxVoltage - MinVoltage);
    }

    /**
     * Sets encoder zero to a specified position.
     * @param position The desired zero position, in degrees
     */
    public synchronized void setZeroPosition(double position) {
        this.zeroPosition = position;
    }


}
