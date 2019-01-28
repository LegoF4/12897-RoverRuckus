package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.constructs.Robot;
import org.firstinspires.ftc.teamcode.constructs.Slides;
import org.firstinspires.ftc.teamcode.utilities.gamepad.AnalogButton;
import org.firstinspires.ftc.teamcode.utilities.gamepad.Button;
import org.firstinspires.ftc.teamcode.utilities.gamepad.DigitalButton;
import org.firstinspires.ftc.teamcode.utilities.gamepad.ToggleButton;
import org.firstinspires.ftc.teamcode.utilities.misc.StaticLog;
import org.firstinspires.ftc.teamcode.utilities.misc.TeleOpMode;

@TeleOp(name="TeleOpMain")
public class TeleOpMain extends TeleOpMode {

    public Robot robot;

    public volatile float powerScalar = 0.65f;
    public volatile boolean intakeOn = false;
    public volatile boolean isOuterControl = true;

    public void runOpMode() throws InterruptedException {
        StaticLog.clearLog();
        robot = new Robot(hardwareMap);
        robot.init();
        /**
         * DRIVE CONTROLS
         */
        //Slow Mode Button
        addButton(new ToggleButton(Key.RIGHT_STICK) {
            @Override
            public void setOutput(int currentState, double value) {
                switch (currentState) {
                    case 0:
                        powerScalar = 0.65f;
                        break;
                    case 1:
                        powerScalar = 0.35f;
                        break;
                }
            }
        });
        //Mecanum drive train code
        addButton(new Button(new Key[]{Key.LEFT_X,Key.LEFT_Y,Key.RIGHT_X}) {
            @Override
            public void update() {
                float gamepad1LeftY = (float) (double) -keyValues.get(1);
                float gamepad1LeftX = (float) (double) keyValues.get(0);
                float gamepad1RightX = (float) (double) keyValues.get(2);

                float FrontLeft = -gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
                float FrontRight = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
                float BackRight = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
                float BackLeft = -gamepad1LeftY + gamepad1LeftX - gamepad1RightX;

                FrontRight = -scaleInput(FrontRight);
                FrontLeft = -scaleInput(FrontLeft);
                BackRight = -scaleInput(BackRight);
                BackLeft = -scaleInput(BackLeft);

                FrontRight = Range.clip(FrontRight, -1, 1);
                FrontLeft = Range.clip(FrontLeft, -1, 1);
                BackLeft = Range.clip(BackLeft, -1, 1);
                BackRight = Range.clip(BackRight, -1, 1);

                getRobot().driveTrain.setPower(-1*powerScalar*FrontLeft, -1*powerScalar*BackLeft, powerScalar*FrontRight, powerScalar*BackRight);
            }
        });
        /**
         * INTAKE CONTROLS
         */
        //Toggles arm for collection
        addButton(new Button(new Key[]{Key.RIGHT_BUMPER, Key.B}) {
            @Override
            public void update() {
                if(this.keyValues.get(0) > 0.5 && this.keyValues.get(1) < 0.5) {
                    isOuterControl = true;
                    getRobot().slides.setArmPosition(Slides.Arm.OUT);
                    getRobot().slides.setIntakeDirection(Slides.Intake.INTAKE);
                } else if (this.keyValues.get(0) < 0.5 && this.keyValues.get(1) < 0.5 && isOuterControl) {
                    getRobot().slides.setArmPosition(Slides.Arm.REST);
                    getRobot().slides.setIntakeDirection(Slides.Intake.STOPPED);
                }
            }
        });
        addButton(new ToggleButton(new Key[]{Key.B, Key.RIGHT_BUMPER}) {
            @Override
            public void setOutput(int currentState, double value) {
                switch (currentState) {
                    case 0:
                        getRobot().slides.setArmPosition(Slides.Arm.REST);
                        getRobot().slides.setIntakeDirection(Slides.Intake.STOPPED);
                        break;
                    case 1:
                        isOuterControl = false;
                        getRobot().slides.setArmPosition(Slides.Arm.IN);
                        getRobot().slides.setIntakeDirection(Slides.Intake.OUTPUT);
                        break;
                }
            }
        });

        /*
         * SLIDE CONTROLS
         */
        addButton(new Button(new TeleOpMode.Key[]{Key.A,Key.Y}) {
            @Override
            public void update() {
                if(keyValues.get(0) > 0.5) {
                    getRobot().slides.setPower(0.82);
                } else if (keyValues.get(1) > 0.5) {
                    getRobot().slides.setPower(-0.82);
                } else {
                    getRobot().slides.setPower(0);
                }
            }
        });
        /**
         * LIFT CODE
         */
        //Maximum lift force
        addButton(new DigitalButton(Key.DPAD_DOWN) {
            @Override
            public void setOutput() {
                getRobot().lift.setPower(1);
            }
        });
        //Slow lift control
        addButton(new Button(new Key[]{Key.DPAD_DOWN, Key.LEFT_TRIGGER, Key.RIGHT_TRIGGER}) {
            @Override
            public void update() {
                if(this.keyValues.get(0) < 0.5) {
                    if(this.keyValues.get(1) > 0.5) {
                        getRobot().lift.setPower(0.4*this.keyValues.get(1));
                    } else if (this.keyValues.get(2) > 0.5) {
                        getRobot().lift.setPower(-0.4*this.keyValues.get(2));
                    } else {
                        getRobot().lift.setPower(0);
                    }
                }
            }
        });
        /*
         * DEPOSITION CONTROL
         */
        //Deposit from middle
        addButton(new AnalogButton(Key.LEFT_BUMPER) {
            @Override
            public void setOutput(double value) {
                if(value > 0.5) {
                    getRobot().setDeposit(Robot.Deposit.DEPOSIT);
                } else {
                    getRobot().setDeposit(Robot.Deposit.MIDDLE);
                }
            }
        });
         /**/
        waitForStart();
        //Loop variables
        while (opModeIsActive()) {
            updateButtons();
            telemetry.addLine("Power Scalar: " + Float.toString(powerScalar));
            telemetry.update();
            Thread.sleep(50);
        }
    }

    public synchronized void setIntakeOn(boolean isIntakeOn) {
        this.intakeOn = isIntakeOn;
    }

    public synchronized boolean getIsIntakeOn() {return this.intakeOn;}

    public synchronized Robot getRobot() {
        return this.robot;
    }

    public float scaleInput(float dVal) {
        float[] scaleArray = {0.0f, 0.05f, 0.09f, 0.10f, 0.12f, 0.15f, 0.18f, 0.24f,
                0.30f, 0.36f, 0.43f, 0.50f, 0.60f, 0.72f, 0.85f, 1.00f, 1.00f};

        // get the corresponding index for the scaleInput array.
        float index = (float) (dVal * 16.0);

        // index should be positive.
        if (index < 0f) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16f) {
            index = 16f;
        }

        // get value from the array.
        float dScale = 0.0f;
        if (dVal < 0f) {
            dScale = -scaleArray[(int) index];
        } else {
            dScale = scaleArray[(int) index];
        }

        // return scaled value.
        return dScale;
    }
}