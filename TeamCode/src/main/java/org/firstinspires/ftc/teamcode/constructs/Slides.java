package org.firstinspires.ftc.teamcode.constructs;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utilities.misc.MathFTC;

/**
 * Created by LeviG on 1/12/2019.
 */

public class Slides {

    public static final double TICKS_PER_ROT = 288;
    public static final double INCHES_PER_ROT = 1.5;

    private volatile HardwareMap map;

    private volatile DcMotor hl;

    private volatile Servo ar;

    private volatile CRServo ir;

    public enum Arm {
        OUT,
        REST,
        IN
    }

    public enum Intake {
        INTAKE,
        STOPPED,
        OUTPUT
    }

    public Slides(HardwareMap map) {
        this.map = map;

        hl = map.get(DcMotor.class, "hl");

        ar = map.get(Servo.class, "ar");
        ir = map.get(CRServo.class, "ir");
    }

    public synchronized void setPower(double power) {
        hl.setPower(power);
    }

    public synchronized int getPosition() {
        return hl.getCurrentPosition();
    }

    public synchronized void setTargetPosition(int position) {
        hl.setTargetPosition(position);
    }

    public synchronized void setIntakeDirection(Intake direction) {
        switch (direction) {
            case OUTPUT:
                setIntakePower(1);
                break;
            case INTAKE:
                setIntakePower(-1);
                break;
            case STOPPED:
                setIntakePower(0);
                break;
            default:
                setIntakePower(0);
                break;
        }
    }

    public synchronized void setIntakePower(double power) {
        ir.setPower(MathFTC.clamp(power, -1, 1));
    }

    public synchronized void setArmPosition(Arm pos) {
        switch (pos) {
            case IN:
                ar.setPosition(0);
                break;
            case OUT:
                ar.setPosition(1);
                break;
            case REST:
                ar.setPosition(0.28);
                break;
        }
    }

    public void prepForEncoders() {
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        synchronized (this) {
            hl.setTargetPosition(hl.getCurrentPosition());
        }
    }

    public synchronized void setArmPosition(double pos) {
        ar.setPosition(pos);
    }

    public synchronized void init() {
        hl.resetDeviceConfigurationForOpMode();
        hl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public synchronized void stop() {
        setPower(0);
        ar.close();
        hl.close();
        ir.close();
    }

    public synchronized void setMotorMode(DcMotor.RunMode runMode) {
        hl.setMode(runMode);
    }
}
