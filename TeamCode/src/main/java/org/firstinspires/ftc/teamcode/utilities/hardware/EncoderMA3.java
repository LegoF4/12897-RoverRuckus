package org.firstinspires.ftc.teamcode.utilities.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.StaticLog;
import org.firstinspires.ftc.teamcode.utilities.misc.MathFTC;

/**
 * Created by LeviG on 9/25/2018.
 */

public class EncoderMA3 extends Encoder {

    //Hardware Parameters
    public static double MaxVoltage = 3.26; //In volts
    public static double MinVoltage = 0.0; //In volts
    public static double threshold = 0.3; //In degrees

    public volatile AnalogInput encoder; //The hardware device

    //Tracking Variables
    private volatile double zeroPosition; //In absolute degrees
    private volatile double priorPosition; //In absolute degrees
    private volatile double currentPosition; //In absolute degrees
    private volatile double firstDerivative; //In degrees per dynamic tick
    private volatile double changePosition;
    private volatile double measuredPosition; //In degrees


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
        //StaticLog.addLine("-----Encoder Cycle-----");
        //StaticLog.addLine("Prior Position: " + Double.toString(priorPosition));
        //StaticLog.addLine("Measured Position: " + Double.toString(measuredPosition));
        if(firstDerivative >= 0 && measuredPosition < priorPosition && Math.abs(measuredPosition - priorPosition) > 90) {
            changePosition = measuredPosition + 360 - priorPosition;
        } else if (firstDerivative <= 0 && measuredPosition > priorPosition && Math.abs(measuredPosition - priorPosition) > 90) {
            changePosition = -1*(priorPosition + 360 - measuredPosition);
        } else if(Math.abs(measuredPosition - priorPosition) > threshold) {
            firstDerivative = Math.signum(measuredPosition - priorPosition);
            changePosition += measuredPosition - priorPosition;
        } else {
            firstDerivative = 0;
        }
        if(Math.abs(changePosition) > 180) {
            changePosition = firstDerivative;
        }
        currentPosition += changePosition;
        priorPosition = measuredPosition;
        //StaticLog.addLine("First Derivative: " + Double.toString(firstDerivative));
        //StaticLog.addLine("Current Position: " + Double.toString(currentPosition-zeroPosition));
        return (currentPosition - zeroPosition); //Corrects for zero position and over-counting
    }

    /**
     * Retrieves the encoders current position
     * @return The number of degrees traversed from the zero position
     */
    public synchronized double getPosition(double measuredPosition) {
        //StaticLog.addLine("-----Encoder Cycle-----");
        //StaticLog.addLine("Prior Position: " + Double.toString(priorPosition));
        //StaticLog.addLine("Measured Position: " + Double.toString(measuredPosition));
        if(firstDerivative >= 0 && measuredPosition < priorPosition && Math.abs(measuredPosition - priorPosition) > 90) {
            currentPosition  += measuredPosition + 360 - priorPosition;
        } else if (firstDerivative <= 0 && measuredPosition > priorPosition && Math.abs(measuredPosition - priorPosition) > 90) {
            currentPosition  -= priorPosition + 360 - measuredPosition;
        } else if(Math.abs(measuredPosition - priorPosition) > threshold) {
            firstDerivative = measuredPosition - priorPosition;
            currentPosition += measuredPosition - priorPosition;
        } else {
            firstDerivative = 0;
        }
        if(Math.abs(currentPosition-priorPosition) > 180) {

        }
        priorPosition = measuredPosition;
        //StaticLog.addLine("First Derivative: " + Double.toString(firstDerivative));
        //StaticLog.addLine("Current Position: " + Double.toString(currentPosition-zeroPosition));
        return (currentPosition - zeroPosition); //Corrects for zero position and over-counting
    }

    public synchronized double getPositionO() {
        double newEncoderAngle = encoder.getVoltage() / 3.26 * 360 + 180 - priorPosition;

        if (newEncoderAngle > 360) {
            newEncoderAngle -= 360;
        }

        if (newEncoderAngle < 0) {
            newEncoderAngle += 360;
        }

        double degreesChanged = newEncoderAngle - 180;

        priorPosition = currentPosition;

        return (degreesChanged / 360);
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

    /**
     * Resets internal encoder device
     */
    public synchronized void reset() {
        this.encoder.resetDeviceConfigurationForOpMode();
    }


}
