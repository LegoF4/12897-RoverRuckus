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

    public volatile float powerScalar = 0.65f; //A power scalar by which the drive train motor powers are scaled
    public volatile boolean intakeOn = false; //True if the intake is supposed to be moving
    public volatile boolean liftIsMoving = false; //True if the user is holding any key that moves the lift
    public static final long wait = 6000; //Pause before setting default intake position
    public volatile long startTime; //Time of game start

    public volatile boolean isHangMode = false; //True when in hang mode

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
        addButton(new Button(new Key[]{Key.RIGHT_BUMPER, Key.LEFT_BUMPER}) {
            @Override
            public void update() {
                if(!isHangMode) {
                    if (this.keyValues.get(0) > 0.5 && this.keyValues.get(1) < 0.5) {
                        getRobot().slides.setArmPosition(Slides.Arm.OUT);
                        getRobot().slides.setIntakeDirection(Slides.Intake.INTAKE);
                    } else if (this.keyValues.get(0) < 0.5 && this.keyValues.get(1) < 0.5 && System.currentTimeMillis() > startTime + wait) {
                        getRobot().slides.setArmPosition(Slides.Arm.REST);
                        getRobot().slides.setIntakeDirection(Slides.Intake.STOPPED);
                    }
                }
            }
        });
        addButton(new DigitalButton(new Key[]{Key.LEFT_BUMPER}) {
            @Override
            public void setOutput() {
                if(!isHangMode) {
                    getRobot().slides.setArmPosition(Slides.Arm.IN);
                    getRobot().slides.setIntakeDirection(Slides.Intake.OUTPUT);
                }
            }
        });
        /**
        * HANG MODE BUTTON
         */
        addButton(new ToggleButton(new Key[]{Key.X}) {
            @Override
            public void setOutput(int currentState, double value) {
                switch (currentState) {
                    case 0:
                        isHangMode = false;
                        break;
                    case 1:
                        isHangMode = true;
                        powerScalar = 0.35f;
                        getRobot().slides.setArmPosition(Slides.Arm.IN);
                        getRobot().setDeposit(Robot.Deposit.HIGH);
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
        addButton(new DigitalButton(new Key[]{Key.DPAD_DOWN, Key.LEFT_STICK}) {
            @Override
            public void setOutput() {
                liftIsMoving = true;
                if(!isHangMode && this.keyValues.get(1) < 0.5) getRobot().setDeposit(Robot.Deposit.MIDDLE);
                getRobot().lift.setPower(1);
            }
        });
        addButton(new DigitalButton(new Key[]{Key.DPAD_UP,Key.LEFT_STICK}) {
            @Override
            public void setOutput() {
                liftIsMoving = true;
                if(!isHangMode && this.keyValues.get(1) < 0.5) getRobot().setDeposit(Robot.Deposit.MIDDLE);
                getRobot().lift.setPower(-1); }
        });
        //Slow lift control
        addButton(new Button(new Key[]{Key.DPAD_DOWN, Key.LEFT_TRIGGER, Key.RIGHT_TRIGGER, Key.DPAD_UP, Key.LEFT_STICK}) {
            @Override
            public void update() {
                if(this.keyValues.get(0) < 0.5 && this.keyValues.get(3) < 0.5) {
                    if(this.keyValues.get(1) > 0.5) {
                        liftIsMoving = true;
                        if(!isHangMode && this.keyValues.get(4) < 0.5) getRobot().setDeposit(Robot.Deposit.MIDDLE);
                        getRobot().lift.setPower(0.4*this.keyValues.get(1));
                    } else if (this.keyValues.get(2) > 0.5) {
                        liftIsMoving = true;
                        if(!isHangMode && this.keyValues.get(4) < 0.5) getRobot().setDeposit(Robot.Deposit.MIDDLE);
                        getRobot().lift.setPower(-0.4*this.keyValues.get(2));
                    } else {
                        liftIsMoving = false;
                        getRobot().lift.setPower(0);
                    }
                }
            }
        });
        /*
         * DEPOSITION CONTROL
         */
        //Deposit from middle
        addButton(new AnalogButton(Key.LEFT_STICK) {
            @Override
            public void setOutput(double value) {
                if(!isHangMode) {
                    if(this.keyValues.get(0) > 0.5) {
                        getRobot().setDeposit(Robot.Deposit.DEPOSIT);
                    } else if (this.keyValues.get(0) < 0.5 && !liftIsMoving) {
                        getRobot().setDeposit(Robot.Deposit.DOWN);
                    }
                }
            }
        });
         /**/
        waitForStart();
        startTime = System.currentTimeMillis();
        //Loop variables
        while (opModeIsActive()) {
            updateButtons();
            telemetry.addLine("Speed: " + (powerScalar > 0.4 ? "FAST" : "SLOW"));
            telemetry.addLine("Hang Mode: " + Boolean.toString(isHangMode));
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