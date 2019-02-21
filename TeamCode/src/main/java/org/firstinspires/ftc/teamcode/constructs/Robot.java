package org.firstinspires.ftc.teamcode.constructs;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by LeviG on 11/16/2018.
 */

public class Robot {

    public volatile DriveTrain driveTrain;
    public volatile Lift lift;
    public volatile Slides slides;
    private volatile HardwareMap map;
    private volatile Deposit deposit;
    private volatile Servo dl;
    private volatile Servo dr;

    public enum Deposit {
        DEPOSIT,
        MIDDLE,
        HIGH,
        DOWN
    }

    public Robot(HardwareMap map) {
        this(map, 0, 0, 0);
    }

    public Robot(HardwareMap map, double x, double y, double phi) {
        this.map = map;
        this.driveTrain = new DriveTrain(map, x, y, phi);
        this.lift = new Lift(map);
        this.slides = new Slides(map);
        this.dl = map.get(Servo.class, "dl");
        this.dr = map.get(Servo.class, "dr");
    }

    public void init() {
        lift.init();
        driveTrain.init();
        slides.init();
    }

    public void stop() {
        lift.stop();
        driveTrain.stop();
        slides.stop();
        dl.close();
        dr.close();
    }

    public void setDeposit(Deposit deposit) {
        if(this.deposit != deposit) {
            this.deposit = deposit;
            if(deposit != null) {
                synchronized (this) {
                    switch (deposit) {
                        case DEPOSIT:
                            dl.setPosition(1);
                            dr.setPosition(0);
                            break;
                        case MIDDLE:
                            dl.setPosition(0.25);
                            dr.setPosition(0.92);
                            break;
                        case DOWN:
                            dl.setPosition(0);
                            dr.setPosition(1);
                            break;
                        case HIGH:
                            dl.setPosition(0.75);
                            dr.setPosition(0.42);
                            break;
                    }
                }
            }
        }
    }
}
