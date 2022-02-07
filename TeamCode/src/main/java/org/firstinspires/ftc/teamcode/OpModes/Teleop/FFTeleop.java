package org.firstinspires.ftc.teamcode.OpModes.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotConfiguration.FreightFrenzy.FreightFrenzyRobot;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.GamepadWrapper;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.TeleopDriver;
//import org.openftc.revextensions2.ExpansionHubEx;
//import org.openftc.revextensions2.ExpansionHubMotor;
//import org.openftc.revextensions2.RevBulkData;

//@Disabled
@TeleOp(name = "FF Goal Teleop", group = "Teleop2020-21")
public class FFTeleop extends OpMode {
    private FreightFrenzyRobot ffRobot;
    //Create joysticks
    private GamepadWrapper joy1 = new GamepadWrapper();
    private GamepadWrapper joy2 = new GamepadWrapper();
    //Create robot driver
    private TeleopDriver teleopDriver;
    //Creates elements for extra data
    //ExpansionHubEx expansionHub;
    //RevBulkData bulkData;
    //ExpansionHubMotor leftf, leftr, rightf, rightr;

    //elevator bools
    boolean initialize_elevator;
    boolean go_down_init;
    boolean new_level;
    boolean manual_override;

    double markerArmPos = 0.5;


    @Override
    public void init() {
        ffRobot = new FreightFrenzyRobot(this.hardwareMap);
        teleopDriver = new TeleopDriver(ffRobot);
        teleopDriver.setMinSpeed(0.2);

        ffRobot.initialize();

        //telemetry output
        //expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 3");
        //leftf = (ExpansionHubMotor) hardwareMap.dcMotor.get("leftf");
        //rightf = (ExpansionHubMotor) hardwareMap.dcMotor.get("rightf");

        //automatic elevator bools
        // initialize_elevator = true;
        go_down_init = true;
        new_level = false;

        //manual elevator bools
        manual_override = false;

        //ffRobot.markerArm.setPosition(0.5);
        //ffRobot.markerHand.setPosition(0.5);
        double armPos = 0;

        telemetry.addData("FlySwatter", ffRobot.flySwatter);
        telemetry.update();
    }

    @Override
    public void loop() {

        //INITIALIZE ELEVATOR
        //check if we are already in middle position
/*
        if ((initialize_elevator && ffRobot.touch_sensors.get(2).isPressed() || manual_override && initialize_elevator)) {
            initialize_elevator = false;
            ffRobot.arm_elevator.setPower(0);
        }

        if (initialize_elevator) {
            ffRobot.arm_elevator.setPower(-1.0);
        }
        */


        //-----------------------------------------------------------------
        //Gamepad 1 is the driver controller, gamepad 2 is the gunner controller

        joy1.update(gamepad1);
        joy2.update(gamepad2);

        //bulkData = expansionHub.getBulkInputData();

        //telemetry.addData("Left Rear", leftf.getVelocity());
        //telemetry.addData("Right Rear", rightf.getVelocity());
        telemetry.addData("FlySwatter", ffRobot.flySwatter);

        //Drivetrain*******************************************************

        if (gamepad1.left_trigger > 0) {
            // control the drive train at full speed
            teleopDriver.setMaxSpeed(0.8f);
        } else if (gamepad1.right_trigger > 0) {
            teleopDriver.setMaxSpeed(0.2f);
        } else {
            // control the drive train at 1/2 speed - Normal driving
            teleopDriver.setMaxSpeed(.6f);
        }

        driveBackwardsToggle(joy1.toggle.back);


        if (gamepad1.a && !gamepad1.x) {
            ffRobot.duckSpinner.setPower(.75);
        } else if (gamepad1.x && !gamepad1.a) {
            ffRobot.duckSpinner.setPower(-.75);
        } else {
            ffRobot.duckSpinner.setPower(0);
        }

        // Arm
        if (gamepad2.dpad_up) {
            ffRobot.flySwatter.armHigh();
        } else if (gamepad2.dpad_left) {
            ffRobot.flySwatter.armMedium();
        } else if (gamepad2.dpad_down) {
            ffRobot.flySwatter.armLow();
        } else if (Math.abs(gamepad2.right_stick_y) > 0.2) {
            ffRobot.flySwatter.arm(-(gamepad2.right_stick_y)); //-(gamepad2.right_stick_y)
        } else {
            ffRobot.flySwatter.armStop();
        }

        // Wrist
        if (gamepad2.left_bumper) {
            ffRobot.flySwatter.wristUp();
        } else if (gamepad2.left_trigger > 0.5) {
            ffRobot.flySwatter.wristDown();
        }

        // Flapper
        if (gamepad2.right_bumper) {
            ffRobot.flySwatter.flapperForward();
        } else if (gamepad2.right_trigger > 0.5) {
            ffRobot.flySwatter.flapperReverse();
        } else {
            ffRobot.flySwatter.flapperStop();
        }

        if (gamepad2.x && !gamepad2.a) {
            ffRobot.extendi.setPower(0.9);
        } else if (gamepad2.a && !gamepad2.x) {
            ffRobot.extendi.setPower(-0.9);
        } else {
            ffRobot.extendi.setPower(0);
        }

    }

    private void driveBackwardsToggle(boolean toggle) {
        if (toggle) {
            teleopDriver.tankDrive(gamepad1, TeleopDriver.Direction.BACKWARD);
            telemetry.addData("Direction", "REVERSED");
        } else {
            teleopDriver.tankDrive(gamepad1, TeleopDriver.Direction.FORWARD);
            telemetry.addData("Direction", "FORWARD");
        }
    }
}

