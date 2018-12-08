package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.constructs.Robot;
import org.firstinspires.ftc.teamcode.utilities.misc.LinearOpMode;

/**
 * Created by LeviG on 12/4/2018.
 */
@Autonomous(name = "Line Test")
public class LineDrive extends LinearOpMode {

    public volatile Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap);
        robot.init();
        waitForStart();
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
