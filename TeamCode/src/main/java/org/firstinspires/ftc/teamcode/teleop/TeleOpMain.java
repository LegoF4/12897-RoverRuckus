package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.constructs.Robot;
import org.firstinspires.ftc.teamcode.constructs.Slides;
import org.firstinspires.ftc.teamcode.utilities.gamepad.AnalogButton;
import org.firstinspires.ftc.teamcode.utilities.gamepad.Button;
import org.firstinspires.ftc.teamcode.utilities.gamepad.DigitalButton;
import org.firstinspires.ftc.teamcode.utilities.gamepad.SingleButton;
import org.firstinspires.ftc.teamcode.utilities.gamepad.ToggleButton;
import org.firstinspires.ftc.teamcode.utilities.misc.StaticLog;
import org.firstinspires.ftc.teamcode.utilities.misc.TeleOpMode;

@TeleOp(name="TeleOpMain")
public class TeleOpMain extends TeleOpMode {

    public Robot robot;
    public volatile long startTime; //Time of game start

    public volatile float powerScalar = 0.65f; //A power scalar by which the drive train motor powers are scaled
    public volatile float turnScalar = 0.75f; //A power scalar by which only the turn power is scaled. This is combined multiplicativly with the powerScalar above.

    public volatile boolean liftIsMoving = false; //True if the user is holding any key that moves the lift

    public volatile boolean autoLiftControl = false; //True if there should be automatic control of the lift
    public volatile boolean autoSlideControl = false; //True if slides are in auto control

    public volatile boolean isDriving = true; //True if the robot should be able to drive

    public volatile boolean intakeInRestMode = true; //True if the intake is in the rest position, with the roller stopped
    public volatile boolean isIntakeIn = false; //True only if the intake is all the way in

    public volatile boolean isHangMode = false; //True when in hang mode

    public volatile Slides.Intake intakeDirection = Slides.Intake.STOPPED;

    public void runOpMode() throws InterruptedException {
        StaticLog.clearLog();
        robot = new Robot(hardwareMap);
        robot.init();

        /*
         * DEPOSITION CONTROL
         */
        //Deposit control
        addButton(new AnalogButton(Key.LEFT_BUMPER_2) {
            @Override
            public void setOutput(double value) {
                if(!isHangMode) {
                    if(this.keyValues.get(0) > 0.5) {
                        getRobot().setDeposit(Robot.Deposit.DEPOSIT);
                        //isDriving = false; Disables driving when dumping.
                    } else if (this.keyValues.get(0) < 0.5) {
                        getRobot().setDeposit(Robot.Deposit.MIDDLE);
                        //isDriving = true; Enables driving when not dunping
                    }
                }
            }
        });

        /**
         * DRIVE CONTROLS
         */
        //Mecanum drive train code
        addButton(new Button(new Key[]{Key.LEFT_X,Key.LEFT_Y,Key.RIGHT_X}) {
            @Override
            public void update() {
                float gamepad1LeftY = (float) (double) -keyValues.get(1);
                float gamepad1LeftX = (float) (double) keyValues.get(0);
                float gamepad1RightX = (float) (double) (turnScalar*keyValues.get(2));

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

                if(isDriving) {
                    getRobot().driveTrain.setPower(-1*powerScalar*FrontLeft, -1*powerScalar*BackLeft, powerScalar*FrontRight, powerScalar*BackRight);
                }
            }
        });
        /**
         * INTAKE CONTROLS
         */
        //Toggles arm for transfer
        addButton(new Button(new Key[]{Key.LEFT_BUMPER}) {
            private boolean isPressed = false;

            @Override
            public void update() {
                if(!isHangMode) {
                    if(this.keyValues.get(0) > 0.5 && !isPressed) { //Makes sure code only runs once per press
                        isPressed = true;
                        if(intakeInRestMode) { //If already in rest position, move to transfer, otherwise, if OUT or IN already, move to REST
                            if(!(liftIsMoving)) { //Only allows user to transfer if lift is still!
                                getRobot().slides.setIntakeDirection(Slides.Intake.INTAKE);
                                intakeDirection = Slides.Intake.INTAKE;
                                getRobot().slides.setArmPosition(Slides.Arm.IN);
                                intakeInRestMode = false;
                                isIntakeIn = true;
                            }
                        } else { //The user should always be able to bring the intake out
                            setIntakeInRestMode(true);
                        }
                    } else if ((!(this.keyValues.get(0) > 0.5)) && isPressed) {
                        isPressed = false;
                    }
                }
            }
        });
        //Toggle arm between intake and rest
        addButton(new Button(new Key[]{Key.RIGHT_BUMPER}) {
            private boolean isPressed = false;

            @Override
            public void update() {
                if(!isHangMode) { //Hang mode overrides these buttons
                    if(this.keyValues.get(0) > 0.5 && !isPressed) { //Ensures code is only run once per press
                        isPressed = true;
                        if(intakeInRestMode) { //Move from rest to intake
                            getRobot().slides.setIntakeDirection(Slides.Intake.INTAKE);
                            intakeDirection = Slides.Intake.INTAKE;
                            getRobot().slides.setArmPosition(Slides.Arm.OUT);
                            intakeInRestMode = false;
                            isIntakeIn = false;
                        } else { //Otherwise, move to rest pose
                            setIntakeInRestMode(true);
                        }
                    } else if ((!(this.keyValues.get(0) > 0.5)) && isPressed) {
                        isPressed = false;
                    }
                }
            }
        });

        //Eject button. Sets rollers to output instead of intake while held, reverts to previous state when released!
        addButton(new Button(new TeleOpMode.Key[]{Key.RIGHT_STICK}) {
            @Override
            public void update() {
                if(!isHangMode) {
                    if(this.keyValues.get(0) > 0.5) {
                        getRobot().slides.setIntakeDirection(Slides.Intake.OUTPUT);
                    } else {
                        getRobot().slides.setIntakeDirection(intakeDirection);
                    }
                }
            }
        });
        /**
        * HANG MODE BUTTON
         */
        //The title above says it all - disables intake control, slows down driving, moves lift to correct height, disables deposition
        addButton(new ToggleButton(new Key[]{Key.X_2}) {

            private boolean isControlling = false;
            private boolean hasBeenDown = false;

            @Override
            public void update() {
                super.update(); //Runs standard ToggleButton update() code
                if(this.isControlling) { //Code to stop moving the lift after it is at the hang position
                    if(getRobot().lift.isUp() && hasBeenDown) {
                        liftIsMoving = false;
                        autoLiftControl = false;
                        getRobot().lift.setPower(0);
                        isControlling = false; //Prevents this loop from running again
                        hasBeenDown = false; //Reset for internal variable for next cycle
                    } else {
                        if(!hasBeenDown) { //Moves lift all the way down, if it has not done so already
                            getRobot().lift.setPower(0.8);
                        } else {
                            getRobot().lift.setPower(-0.3); //Moves lift up after full decent
                        }
                        if(getRobot().lift.isDown()) {
                            hasBeenDown = true; //Checks if lift is down; if so, start moving up on next TeleOp cycle
                        }
                    }
                }
            }

            @Override
            public void setOutput(int currentState, double value) {
                switch (currentState) {
                    case 0:
                        isHangMode = false;
                        powerScalar = 0.8f; //Resets scalars
                        turnScalar = 0.75f;
                        isControlling = false; //Releases internal button control
                        autoLiftControl = false;
                        getRobot().slides.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //Frees slides to move
                        getRobot().slides.setPower(0); //Sets slide power to 0.00 from 0.85
                        getRobot().lift.setPower(0); //Stops lift
                        break;
                    case 1:
                        isHangMode = true;
                        powerScalar = 0.35f; //Slows driving
                        turnScalar = 1.0f; //Speeds up turning relative to driving, but still slower than normal turning
                        //Sets intake to rest mode
                        setIntakeInRestMode(true);
                        getRobot().setDeposit(Robot.Deposit.MIDDLE); //Ensures deposit bucket won't interfere
                        getRobot().slides.setIntakeDirection(Slides.Intake.STOPPED);
                        intakeDirection = Slides.Intake.STOPPED;
                        isControlling = true;
                        autoLiftControl = true;
                        liftIsMoving = true;
                        getRobot().slides.prepForEncoders(); //Turns on slide encoder, set target to current position, sets holding power to 0.85
                        break;
                }
            }
        });

        /*
         * SLIDE CONTROLS
         */
        //Basic slide in/out controls
        addButton(new Button(new TeleOpMode.Key[]{Key.A,Key.Y}) {
            @Override
            public void update() {
                if(keyValues.get(0) > 0.5) {
                    autoSlideControl = false;
                    getRobot().slides.setPower(0.82);
                } else if (keyValues.get(1) > 0.5) {
                    autoSlideControl = false;
                    getRobot().slides.setPower(-0.82);
                } else {
                    if(!autoSlideControl) getRobot().slides.setPower(0);
                }
            }
        });
        /**
         * LIFT CODE
         */
        //Moves lift to silver/hang hiehgt
        addButton(new Button(new Key[]{Key.A_2}) {

            private boolean isControlling = false;
            private boolean hasBeenDown = false;
            private boolean isPressed = false;

            @Override
            public void update() {
                if(!isHangMode) {
                    if(this.keyValues.get(0) > 0.5 && !isPressed) {
                        if(!autoLiftControl) { //Ensures another auto sequence isn't already in affect
                            isPressed = true;
                            isControlling = true;
                            liftIsMoving = true;
                            autoLiftControl = true;
                        }
                    } else if (this.keyValues.get(0) < 0.5 && isPressed) {
                        isPressed = false;
                    }
                }
                if(this.isControlling) { //Code to stop moving the lift after it is at the hang position
                    if((getRobot().lift.isUp() && hasBeenDown) || !autoLiftControl) { //If autoLiftControl terminated due to user input, stops
                        liftIsMoving = false;
                        autoLiftControl = false;
                        getRobot().lift.setPower(0);
                        isControlling = false; //Prevents this loop from running again
                        hasBeenDown = false; //Reset for internal variable for next cycle
                    } else {
                        if(!hasBeenDown) { //Moves lift all the way down, if it has not done so already
                            getRobot().lift.setPower(0.8);
                        } else {
                            getRobot().lift.setPower(-0.8); //Moves lift up after full decent
                        }
                        if(getRobot().lift.isDown()) {
                            hasBeenDown = true; //Checks if lift is down; if so, start moving up on next TeleOp cycle
                        }
                    }
                }
            }
        });

        //Drops lift all the way down whenever the right bumper is pressed
        addButton(new Button(new TeleOpMode.Key[]{Key.RIGHT_BUMPER}) {

            private boolean isControlling = false;
            private boolean isPressed = false;

            @Override
            public void update() {
                if(!isHangMode) {
                    if(this.keyValues.get(0) > 0.5 && !isPressed) {
                        if(!autoLiftControl) {
                            isPressed = true;
                            isControlling = true;
                            autoLiftControl = true;
                            liftIsMoving = true;
                        }
                    } else if (this.keyValues.get(0) < 0.5 && isPressed) {
                        isPressed = false;
                    }
                    if(this.isControlling) {
                        if(getRobot().lift.isDown() || !autoLiftControl) {
                            liftIsMoving = false;
                            autoLiftControl = false;
                            getRobot().lift.setPower(0);
                            isControlling = false;
                        } else {
                            getRobot().lift.setPower(0.75);
                        }
                    }
                }
            }
        });

        //Maximum lift force down
        addButton(new DigitalButton(new Key[]{Key.DPAD_DOWN_2, Key.LEFT_BUMPER_2}) {
            @Override
            public void setOutput() {
                autoLiftControl = false;
                setLiftIsMoving(true);
                if(!isHangMode && this.keyValues.get(1) < 0.5) getRobot().setDeposit(Robot.Deposit.MIDDLE);
                getRobot().lift.setPower(1);
            }
        });
        //Maximum lift force up
        addButton(new DigitalButton(new Key[]{Key.DPAD_UP_2,Key.LEFT_BUMPER_2}) {
            @Override
            public void setOutput() {
                autoLiftControl = false;
                setLiftIsMoving(true);
                if(!isHangMode && this.keyValues.get(1) < 0.5) getRobot().setDeposit(Robot.Deposit.MIDDLE);
                getRobot().lift.setPower(-1); }
        });
        // Analog lift control
        addButton(new Button(new Key[]{Key.DPAD_DOWN_2, Key.LEFT_TRIGGER_2, Key.RIGHT_TRIGGER_2, Key.DPAD_UP_2, Key.LEFT_BUMPER_2}) {
            @Override
            public void update() {
                if(this.keyValues.get(0) < 0.5 && this.keyValues.get(3) < 0.5) {
                    if(this.keyValues.get(1) > 0.5) {
                        autoLiftControl = false;
                        setLiftIsMoving(true);
                        if(!isHangMode && this.keyValues.get(4) < 0.5) getRobot().setDeposit(Robot.Deposit.MIDDLE);
                        getRobot().lift.setPower(1*this.keyValues.get(1));
                    } else if (this.keyValues.get(2) > 0.5) {
                        autoLiftControl = false;
                        setLiftIsMoving(true);
                        if(!isHangMode && this.keyValues.get(4) < 0.5) getRobot().setDeposit(Robot.Deposit.MIDDLE);
                        getRobot().lift.setPower(-1*this.keyValues.get(2));
                    } else {
                        if(!autoLiftControl) {
                            setLiftIsMoving(false);
                            getRobot().lift.setPower(0);
                        }
                    }
                }
            }
        });

         /**/
        waitForStart();
        startTime = System.currentTimeMillis();
        setIntakeInRestMode(true);
        //Loop variables
        while (opModeIsActive()) {
            updateButtons();
            telemetry.addLine("Speed: " + (powerScalar > 0.4 ? "FAST" : "SLOW"));
            telemetry.addLine("Hang Mode: " + (isHangMode ? "TRUE" : "FALSE"));
            telemetry.update();
            Thread.sleep(20);
        }
    }

    //Purely to avoid excessive C/Ping of the following case of the true condition
    public synchronized void setIntakeInRestMode(boolean intakeInRestMode) {
        this.intakeInRestMode = intakeInRestMode; //This ensures the LEFT_BUMPER and RIGHT_BUMPER controls know whether to move "out" or to rest
        if(this.intakeInRestMode) { //Just methodizes setting the full rest pose parameters
            getRobot().slides.setArmPosition(Slides.Arm.REST);
            getRobot().slides.setIntakeDirection(Slides.Intake.INTAKE);
            intakeDirection = Slides.Intake.INTAKE;
            isIntakeIn = false;
        }
    }

    //Only use for case of true....
    public synchronized void setLiftIsMoving(boolean liftIsMoving) {
        this.liftIsMoving = liftIsMoving;
        if(this.liftIsMoving && isIntakeIn) {
            setIntakeInRestMode(true); //The intake cannot both be in and the lift is moving. The lift takes precedence, and the intake moves out.
        }
    }

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
        float dScale;
        if (dVal < 0f) {
            dScale = -scaleArray[(int) index];
        } else {
            dScale = scaleArray[(int) index];
        }

        // return scaled value.
        return dScale;
    }
}