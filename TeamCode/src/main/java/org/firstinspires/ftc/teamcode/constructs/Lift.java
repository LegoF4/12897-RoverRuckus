package org.firstinspires.ftc.teamcode.constructs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.controllers.Controller;
import org.firstinspires.ftc.teamcode.utilities.misc.StaticLog;

/**
 * Created by LeviG on 12/16/2018.
 */

public class Lift {

    public static final double ticksToDescend = 1000;

    private volatile Controller controller;

    private DcMotor v1;
    private DcMotor v2;

    public Lift(HardwareMap map) {
        v1 = map.get(DcMotor.class,"v1");
        v2 = map.get(DcMotor.class,"v2");
    }

    public synchronized void setPower(double power) {
        v1.setPower(power);
        v2.setPower(-power);
    }

    public synchronized void init() {
        v1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        v2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        v1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        v2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public synchronized void stopMoving() {
        if(controller != null) controller.stopControl();
        setPower(0);
    }

    public synchronized void stop() {
        stopMoving();
        setPower(0);
    }

    public synchronized void descend() {
        controller = new DescendThread();
        controller.startControl();
    }

    private class DescendThread extends Controller {
        @Override
        public void run() {
            double s1 = v1.getCurrentPosition();
            double s2 = v2.getCurrentPosition();
            while(!(v1.getCurrentPosition() > s1 + ticksToDescend) || !(v2.getCurrentPosition() < s2 - ticksToDescend)) {
                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    StaticLog.addLine("Descent Interrupted. Terminating. ");
                }
            }
            setPower(0);
        }

        @Override
        public void setOutput(double u) {
            setPower(u);
        }
    }
}
