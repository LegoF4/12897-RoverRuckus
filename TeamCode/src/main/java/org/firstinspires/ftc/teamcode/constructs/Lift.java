package org.firstinspires.ftc.teamcode.constructs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by LeviG on 12/16/2018.
 */

public class Lift {

    private DcMotor vl;
    private DcMotor vr;

    public Lift(HardwareMap map) {
        vl = map.get(DcMotor.class,"vl");
        vr = map.get(DcMotor.class,"vr");
    }

    public synchronized void setPower(double power) {
        vl.setPower(power);
        vr.setPower(-power);
    }

    public synchronized void init() {
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
