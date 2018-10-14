package org.firstinspires.ftc.teamcode.debug;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.StaticLog;
import org.firstinspires.ftc.teamcode.utilities.hardware.Encoder;
import org.firstinspires.ftc.teamcode.utilities.hardware.EncoderMA3;
import org.firstinspires.ftc.teamcode.utilities.misc.LinearOpMode;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by LeviG on 9/30/2018.
 */
@TeleOp(name = "Encoder Test")
public class EncoderTest extends LinearOpMode{

    EncoderMA3 encoder;

    @Override
    public void runOpMode() throws InterruptedException{
        StaticLog.clearLog();
        this.encoder = new EncoderMA3(this.hardwareMap.get(AnalogInput.class, "center"));
        Thread.sleep(200);
        this.encoder.setZeroPosition();
        Thread.sleep(100);
        waitForStart();
        int count = 0;
        boolean run = true;
        List<Double> vs = new ArrayList<Double>();
        List<Double> as = new ArrayList<Double>();
        long startTime = System.currentTimeMillis();
        while(opModeIsActive() && run) {
            double measuredPosition = 360*encoder.encoder.getVoltage()/(encoder.MaxVoltage - encoder.MinVoltage);
            vs.add(measuredPosition);
            as.add(encoder.getPosition(measuredPosition));
            count++;
            //if(count % 10 == 0) {
              //  telemetry.addLine("Traversed Degrees: " + Double.toString(encoder.getPosition()));
             //   telemetry.addLine("Voltage: " + Double.toString(encoder.encoder.getVoltage()));
             //   telemetry.update();
           // }
            if(count > 750) {
                run = false;
            }
            //encoder.getPosition();
            Thread.sleep(20);
        }
        this.telemetry.addLine("Elapsed Time: " + Double.toString((System.currentTimeMillis()-startTime)/1000));
        telemetry.update();
        for (Double v : vs) {
            StaticLog.addLine("V " + Double.toString(v));
        }
        for(Double a : as) {
            StaticLog.addLine("A " + Double.toString(a));
        }
    }
}
