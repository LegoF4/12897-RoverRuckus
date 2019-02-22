package org.firstinspires.ftc.teamcode.vision.dogecv;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.OpenCVPipeline;
import com.disnodeteam.dogecv.detectors.DogeCVDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldDetector;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Size;

/**
 * Created by LeviG on 1/20/2019
 *
 * This class is useful for finding the size you want to crop your camera image to.
 */
@TeleOp(name = "Cropping Example", group = "DogeCV")
public class CroppingExample extends OpMode {

    //This is an example of an anonymous implementation of the DogeCV Detector
    private GoldDetector detector;

    @Override
    public void init() {
        detector = new GoldDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();// Optional Tuning

        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.PERFECT_AREA; // Can also be PERFECT_AREA
        detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.perfectAreaScorer.weight = 0.05;
        detector.cropTLCorner = new Point(200, 15); //Sets the top left corner of the new image, in pixel (x,y) coordinates
        detector.cropBRCorner = new Point(500, 450); //Sets the bottom right corner of the new image, in pixel (x,y) coordinates


        telemetry.addData("Status", "DogeCV 2019.1 - Cropping Example");

        // Set up detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera

        detector.enable();
    }
    /*
     * Code to run REPEATEDLY when the driver hits INIT
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY when the driver hits PLAY
     */
    @Override
    public void loop() {
        telemetry.addLine("DogeCV 2019.1 - Cropping Example");
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        if(detector != null) detector.disable(); //Make sure to run this on stop!
    }
}
