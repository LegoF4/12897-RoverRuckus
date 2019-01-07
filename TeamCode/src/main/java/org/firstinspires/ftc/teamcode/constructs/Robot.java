package org.firstinspires.ftc.teamcode.constructs;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by LeviG on 11/16/2018.
 */

public class Robot {

    public volatile DriveTrain driveTrain;
    public volatile Lift lift;
    private volatile HardwareMap map;

    public static final double markerDown = 0.2;
    public static final double markerUp = 0.8;

    public Robot(HardwareMap map) {
        this.map = map;
        this.driveTrain = new DriveTrain(map);
        this.lift = new Lift(map);
    }

    public void init() {
        lift.init();

    }

    public void stop() {
        lift.stop();
        driveTrain.stop();
    }

    //public void deployMarker() {
       /// teamMarker.setPosition(markerDown);
   // }

   // public void retractMarker() {
   //     teamMarker.setPosition(markerUp);
   // }
}
