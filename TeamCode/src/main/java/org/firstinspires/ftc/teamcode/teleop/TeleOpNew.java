package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.constructs.Robot;
import org.firstinspires.ftc.teamcode.constructs.Slides;
import org.firstinspires.ftc.teamcode.utilities.gamepad.Button;
import org.firstinspires.ftc.teamcode.utilities.gamepad.ConditionedDigitalButton;
import org.firstinspires.ftc.teamcode.utilities.gamepad.DigitalButton;
import org.firstinspires.ftc.teamcode.utilities.gamepad.ToggleButton;
import org.firstinspires.ftc.teamcode.utilities.misc.LinearOpMode;
import org.firstinspires.ftc.teamcode.utilities.misc.StaticLog;
import org.firstinspires.ftc.teamcode.utilities.misc.TeleOpMode;

@TeleOp(name="TeleOpNew")
public class TeleOpNew extends TeleOpMode {

    public Robot robot;

    public volatile float powerScalar = 0.55f;

    public void runOpMode() throws InterruptedException {
        StaticLog.clearLog();
        Robot robot = new Robot(hardwareMap);
        /**
         * DRIVE CODE
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
         * INTAKE CODE
         */
        //Toggles arm for collection
        addButton(new ToggleButton(Key.RIGHT_BUMPER, 0.5d) {
            @Override
            public void setOutput(int currentState, double value) {
                switch (currentState) {
                    case 0:
                        getRobot().slides.setArmPosition(Slides.Arm.REST);
                        break;
                    case 1:
                        getRobot().slides.setArmPosition(Slides.Arm.OUT);
                        break;
                }
            }
        });
        //Rotates arm in to transfer
        addButton(new DigitalButton(Key.B) {
            @Override
            public void setOutput() {
                getRobot().slides.setArmPosition(Slides.Arm.IN);
            }
        });
        addButton(new ConditionedDigitalButton(new TeleOpMode.Key[]{Key.A,Key.Y}) {
            @Override
            public void setOutput(int state) {
                switch (state) {
                    case 0:
                        getRobot().slides.setPower(0.5);
                        break;
                    case 1:
                        getRobot().slides.setPower(-0.5);
                        break;
                    default:
                        getRobot().slides.setPower(0);
                        break;
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
        //Variable lift force
        addButton(new Button(new Key[]{Key.LEFT_TRIGGER, Key.RIGHT_TRIGGER, Key.DPAD_DOWN}) {
            @Override
            public void update() {
                if(!(keyValues.get(2) > 0.5)) {
                    double liftPower = keyValues.get(1) > 0.05 ? keyValues.get(1) : -1*keyValues.get(0);
                    liftPower = 0.35*Math.signum(liftPower)*Math.pow(liftPower,2);
                    getRobot().lift.setPower(-liftPower);
                }
            }
        });
        //Deposit from middle
        addButton(new ToggleButton(Key.LEFT_BUMPER) {
            @Override
            public void setOutput(int currentState, double value) {
                switch (currentState) {
                    case 0:
                        getRobot().setDeposit(Robot.Deposit.MIDDLE);
                        break;
                    case 1:
                        getRobot().setDeposit(Robot.Deposit.DEPOSIT);
                        break;
                }
            }
        });
        //Move deposit bucket down for transfer, if not dumping
        addButton(new ToggleButton(new TeleOpMode.Key[]{Key.X, Key.LEFT_BUMPER}) {
            @Override
            public void setOutput(int currentState, double value) {
                if(!(keyValues.get(1) > 0.5)) {
                    switch (currentState) {
                        case 0:
                            getRobot().setDeposit(Robot.Deposit.MIDDLE);
                            break;
                        case 1:
                            getRobot().setDeposit(Robot.Deposit.DOWN);
                            break;
                    }
                }
            }
        });
        robot.init();
        waitForStart();
        //Loop variables
        while (opModeIsActive()) {
            updateButtons();
            Thread.sleep(50);
        }
    }

    public Robot getRobot() {
        return robot;
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

