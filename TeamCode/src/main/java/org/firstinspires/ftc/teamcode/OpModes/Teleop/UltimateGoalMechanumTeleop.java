package org.firstinspires.ftc.teamcode.OpModes.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotConfiguration.UltimateGoal.UltimateGoalRobot;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.ContinuousServo;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.GamepadWrapper;
import org.firstinspires.ftc.teamcode.Subsystems.ReversableMotor;
//import org.firstinspires.ftc.teamcode.RobotCoreExtensions.TeleopDriver;


/**
 * Created by 3486 on 10/10/2019.
 */

@TeleOp(name = "Ultimate Goal Teleop", group = "Teleop2020")
public class UltimateGoalMechanumTeleop extends OpMode {
    //Declare parts of the robot that will be used by this Teleop
    // private UltimateGoalRobot ultimateGoalRobot = new UltimateGoalRobot(this.hardwareMap);
    //Create joysticks
    private GamepadWrapper joy1 = new GamepadWrapper();
    private GamepadWrapper joy2 = new GamepadWrapper();
    // private TeleopDriver teleopDriver;
    private static final float CLOCKWISE_TURNING_SENSITIVITY = 0.75f;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    public ReversableMotor flapper;
    public ReversableMotor belt;
    public ReversableMotor delivery1;
    public ReversableMotor delivery2;

    public ContinuousServo ringServo;

    /*public ReversableMotor flapper;
    public ReversableMotor belt;
*/
    private final double lowSpeedMultiplier = 0.25;
    private final double halfSpeedMultiplier = 0.5;
    private final double fullSpeedMultiplier = 1;

    @Override
    public void init() {
        //UltimateGoalRobot ultimateGoalRobot = new UltimateGoalRobot(this.hardwareMap);
        //ultimateGoalRobot.initialize();
        // teleopDriver = new TeleopDriver(ultimateGoalRobot);
        // teleopDriver.setMinSpeed(0.2);
        // ultimateGoalRobot.initialize();
        //skystoneRobot.releaseServo.open();
        frontLeft = hardwareMap.get(DcMotor.class, "leftf");
        frontRight = hardwareMap.get(DcMotor.class, "rightf");
        backLeft = hardwareMap.get(DcMotor.class, "leftr");
        backRight = hardwareMap.get(DcMotor.class, "rightr");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);


/*
        final Servo armAngleServo = hardwareMap.servo.get("armAngle");
        this.armAngleServo = new OpenCloseServo(armAngleServo, 0.1, .99, 0.1);
        //Capstone Servo
        final Servo armGripServo = hardwareMap.servo.get("armGrip");
        this.armGripServo = new OpenCloseServo(armGripServo, .01, .99, .05);

        final Servo ringServo = hardwareMap.servo.get("ringServo");
        this.ringServo = new ContinuousServo(ringServo);
        */
        final DcMotor delivery1 = hardwareMap.dcMotor.get("delivery1");
        this.delivery1 = new ReversableMotor(delivery1, 1);

        final DcMotor delivery2 = hardwareMap.dcMotor.get("delivery2");
        this.delivery2 = new ReversableMotor(delivery2, 1);

        final DcMotor flapper = hardwareMap.dcMotor.get("flapper");
        this.flapper = new ReversableMotor(flapper, 1);

        final DcMotor belt = hardwareMap.dcMotor.get("belt");
        this.belt = new ReversableMotor(belt, 1);

        final Servo ringServo = hardwareMap.servo.get("ringServo");
        this.ringServo = new ContinuousServo(ringServo);
    }

    @Override
    public void loop() {
        //Gamepad 1 is the driver controller, gamepad 2 is the gunner controller
        joy1.update(gamepad1);
        joy2.update(gamepad2);

        //Drivetrain*******************************************************
/*
        //Toggle Half Speed on the drivetrain
        if (gamepad1.left_trigger > 0) {
            // control the drive train at full speed
            teleopDriver.setMaxSpeed(1f);
        } else if (gamepad1.right_trigger>0){
            teleopDriver.setMaxSpeed(.3f);
        }

        else {
            // control the drive train at 1/2 speed - Normal driving
            teleopDriver.setMaxSpeed(.5f);
        }

        driveBackwardsToggle(joy1.toggle.back);
*/
        // Create direction vector
        final float forward = -gamepad1.left_stick_y;
        final float right = gamepad1.left_stick_x;
        float clockwise = gamepad1.right_stick_x;

        // Now add a tuning constant K for the “rotate” axis sensitivity.
        // Start with K=0, and increase it very slowly (do not exceed K=1)
        // to find the right value after you’ve got fwd/rev and strafe working:
        clockwise = CLOCKWISE_TURNING_SENSITIVITY * clockwise;

        // Now apply the inverse kinematic tranformation
        // to convert your vehicle motion command
        // to 4 wheel speed commands:
        float frontLeftSpeed = forward + clockwise - right;
        float frontRightSpeed = forward - clockwise + right;
        float backLeftSpeed = forward + clockwise + right;
        float backRightSpeed = forward - clockwise - right;

        // Finally, normalize the wheel speed commands
        // so that no wheel speed command exceeds magnitude of 1:
        float max = Math.abs(frontLeftSpeed);
        if (Math.abs(frontRightSpeed) > max) max = Math.abs(frontRightSpeed);
        if (Math.abs(backLeftSpeed) > max) max = Math.abs(backLeftSpeed);
        if (Math.abs(backRightSpeed) > max) max = Math.abs(backRightSpeed);
        if (max > 1) {
            frontLeftSpeed /= max;
            frontRightSpeed /= max;
            backLeftSpeed /= max;
            backRightSpeed /= max;
        }

        // Send power to wheels

        if (gamepad1.left_trigger > 0) {
            frontLeft.setPower(frontLeftSpeed * fullSpeedMultiplier);
            frontRight.setPower(frontRightSpeed * fullSpeedMultiplier);
            backLeft.setPower(backLeftSpeed * fullSpeedMultiplier);
            backRight.setPower(backRightSpeed * fullSpeedMultiplier);
        } else if (gamepad1.right_trigger > 0) {
            frontLeft.setPower(frontLeftSpeed * lowSpeedMultiplier);
            frontRight.setPower(frontRightSpeed * lowSpeedMultiplier);
            backLeft.setPower(backLeftSpeed * lowSpeedMultiplier);
            backRight.setPower(backRightSpeed * lowSpeedMultiplier);
        } else {
            frontLeft.setPower(frontLeftSpeed * halfSpeedMultiplier);
            frontRight.setPower(frontRightSpeed * halfSpeedMultiplier);
            backLeft.setPower(backLeftSpeed * halfSpeedMultiplier);
            backRight.setPower(backRightSpeed * halfSpeedMultiplier);
        }
        if (gamepad1.dpad_up) {
            frontLeft.setPower(0.99);
            frontRight.setPower(0.99);
            backLeft.setPower(0.99);
            backRight.setPower(0.99);
        } else if (gamepad1.dpad_down) {
            frontLeft.setPower(-0.99);
            frontRight.setPower(-0.99);
            backLeft.setPower(-0.99);
            backRight.setPower(-0.99);
        } else if (gamepad1.dpad_left) {
            frontLeft.setPower(0.99);
            frontRight.setPower(-0.99);
            backLeft.setPower(-0.99);
            backRight.setPower(0.99);
            // Right side is inverted, change to opposite signs
        } else if (gamepad1.dpad_right) {
            frontLeft.setPower(-0.99);
            frontRight.setPower(-0.99);
            backLeft.setPower(0.99);
            backRight.setPower(0.99);
        }
        if (gamepad1.b) {
            driveRight(2000, 1);
        }
        if (gamepad1.x) {
            driveLeft(2000, 1);
        }/*
        if (gamepad1.left_bumper) {
            driveLeftHold(1);
        }
        if (gamepad1.right_bumper) {
            driveRightHold(1);
        }*/
        if (gamepad1.back) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }


    //PICKUP Functions*******************************************************


    //Pickup Motor
        if(gamepad1.right_bumper) {
        flapper.run(.99);
        belt.run(.99);
    } else if(gamepad1.y) {
        flapper.reverse(-.99);
        belt.reverse(-.99);
    } else if(gamepad1.left_bumper) {
        flapper.stop();
        belt.stop();
    }

    //Delivery Functions **********************************************
        if (joy2.toggle.right_bumper) {
        delivery1.reverse(-0.99);
        delivery2.run(0.99);
        ringServo.setPower(-0.99);
    } else {
        delivery1.stop();
        delivery2.stop();
        ringServo.setPower(0);
    }
/*
        if (joy1.toggle.right_bumper) {
        ultimateGoalRobot.armAngleServo.open();
    } else {
        ultimateGoalRobot.armAngleServo.close();
    }

        if (joy2.toggle.left_bumper) {
        ultimateGoalRobot.armGripServo.open();
    } else{
        ultimateGoalRobot.armGripServo.close();
    }*/
/*
        if(joy2.toggle.y) {
            ultimateGoalRobot.ringServo.setPower(.9);
        }else {
            ultimateGoalRobot.ringServo.setPower(0);
        }
*/
    }
    public void driveLeft(double distance, double inputPower) {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setPower(inputPower);
        frontRight.setPower(-inputPower);
        backLeft.setPower(-inputPower);
        backRight.setPower(inputPower);

        while(backLeft.getCurrentPosition() >= -distance){ }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void driveRight(double distance, double inputPower) {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Right side signs are incorrect change later
        frontLeft.setPower(-inputPower);
        frontRight.setPower(-inputPower);
        backLeft.setPower(inputPower);
        backRight.setPower(inputPower);

        while(backLeft.getCurrentPosition() <= distance){ }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
    public void driveLeftHold(double inputPower) {

        frontLeft.setPower(inputPower);
        frontRight.setPower(-inputPower);
        backLeft.setPower(-inputPower * .8);
        backRight.setPower(inputPower * .8);
    }
    public void driveRightHold(double inputPower) {
        frontLeft.setPower(-inputPower);
        frontRight.setPower(inputPower);
        backLeft.setPower(inputPower * .8);
        backRight.setPower(-inputPower * .8);
    }

}



   /* private void driveBackwardsToggle(boolean toggle) {
        if (toggle) {
            teleopDriver.tankDrive(gamepad1, TeleopDriver.Direction.FORWARD);
        } else {
            teleopDriver.tankDrive(gamepad1, TeleopDriver.Direction.BACKWARD);
        }
    }
}*/