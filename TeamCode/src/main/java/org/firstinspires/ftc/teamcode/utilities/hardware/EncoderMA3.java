package org.firstinspires.ftc.teamcode.utilities.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.utilities.misc.MathFTC;

/**
 * Created by LeviG on 9/25/2018.
 */

public class EncoderMA3 extends Encoder {

    //Hardware Parameters
    public static double MaxVoltage = 3.6; //In volts
    public static double MinVoltage = 0.0; //In volts
    public static double threshold = 0.01; //In degrees

    private volatile AnalogInput encoder; //The hardware device

    //Tracking Variables
    private volatile double zeroPosition; //In absolute degrees
    private volatile double priorPosition; //In absolute degrees
    private volatile double currentPosition; //In absolute degrees
    private volatile double firstDerivative; //Parity only
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

        if(firstDerivative > 0 && measuredPosition < MathFTC.simplifyDeg(priorPosition)) {
            currentPosition  += measuredPosition + 360 - MathFTC.simplifyDeg(priorPosition);
        } else if (firstDerivative < 0 && measuredPosition > MathFTC.simplifyDeg(priorPosition)) {
            currentPosition  -= MathFTC.simplifyDeg(priorPosition) + 360 - measuredPosition;
        } else if(Math.abs(currentPosition - priorPosition) > threshold) {
            currentPosition += measuredPosition;
            firstDerivative = Math.signum(currentPosition - priorPosition);
        } else {
            priorPosition = currentPosition;
            firstDerivative = 0;
        }
        return (currentPosition - zeroPosition); //Corrects for zero position and over-counting
    }

    /**
     * Sets encoder zero position to current position.
     */
    public synchronized void setZeroPosition() {
        this.encoder.resetDeviceConfigurationForOpMode();
        this.zeroPosition = 360*encoder.getVoltage()/(MaxVoltage - MinVoltage);
    }

    /**
     * Sets encoder zero to a specified position.
     * @param position The desired zero position, in degrees
     */
    public synchronized void setZeroPosition(double position) {
        this.encoder.resetDeviceConfigurationForOpMode();
        this.zeroPosition = position;
    }


}
