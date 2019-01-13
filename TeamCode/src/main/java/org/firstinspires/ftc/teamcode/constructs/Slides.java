package org.firstinspires.ftc.teamcode.constructs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by LeviG on 1/12/2019.
 */

public class Slides {

    private volatile HardwareMap map;

    private volatile DcMotor hl;
    private volatile DcMotor hr;

    private volatile Servo ar;

    public enum Arm {
        OUT,
        REST,
        IN
    }

    public Slides(HardwareMap map) {
        this.map = map;

        hl = map.get(DcMotor.class, "hl");
        hr = map.get(DcMotor.class, "hr");

        ar = map.get(Servo.class, "ar");
    }

    public synchronized void setPower(double power) {
        hl.setPower(power);
        hr.setPower(-power);
    }

    public void setArmPosition(Arm pos) {
        switch (pos) {
            case OUT:
                setArmPosition(0.97);
            case IN:
                setArmPosition(0.03);
            case REST:
                setArmPosition(0.5);
        }
    }

    public synchronized void setArmPosition(double pos) {
        ar.setPosition(pos);
    }

    public synchronized void init() {
        hl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public synchronized void stop() {
        setPower(0);
        ar.close();
        hl.close();
        hr.close();
    }
}
