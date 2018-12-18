package org.firstinspires.ftc.teamcode.constructs;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by LeviG on 11/16/2018.
 */

public class Robot {

    public volatile DriveTrain driveTrain;
    public volatile Lift lift;
    private volatile HardwareMap map;

    public Robot(HardwareMap map) {
        this.map = map;
        this.driveTrain = new DriveTrain(map);
        this.lift = new Lift(map);
    }

    public void init() {
        driveTrain.init();
        lift.init();
    }

    public void stop() {
        lift.stop();
        driveTrain.stop();
    }
}
