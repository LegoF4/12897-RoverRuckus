package org.firstinspires.ftc.teamcode.constructs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by LeviG on 12/16/2018.
 */

public class Lift {

    private DcMotor vl;
    private DcMotor vr;
    private DigitalChannel liftUp;
    private DigitalChannel liftDown;

    public Lift(HardwareMap map) {
        vl = map.get(DcMotor.class,"vl");
        vr = map.get(DcMotor.class,"vr");
        liftUp = map.get(DigitalChannel.class, "limit1");
        liftDown = map.get(DigitalChannel.class, "liftLimit");
        liftUp.setMode(DigitalChannel.Mode.INPUT);
        liftDown.setMode(DigitalChannel.Mode.INPUT);
    }

    public synchronized Boolean isUp() {
        return !liftUp.getState();
    }

    public synchronized Boolean isDown() {
        return !liftDown.getState();
    }

    public synchronized void setPower(double power) {
        vl.setPower(-power);
        vr.setPower(-power);
    }

    public synchronized void init() {
        liftUp.resetDeviceConfigurationForOpMode();
        vl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public synchronized void stopMoving() {
        setPower(0);
    }

    public synchronized void stop() {
        stopMoving();
        vl.close();
        vr.close();
    }
}
