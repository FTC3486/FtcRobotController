package org.firstinspires.ftc.teamcode.OpModes.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotConfiguration.FreightFrenzy.FreightFrenzyRobot;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.GamepadWrapper;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.TeleopDriver;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

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
    ExpansionHubEx expansionHub;
    RevBulkData bulkData;
    ExpansionHubMotor leftf, leftr, rightf, rightr;

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
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 3");
        leftf = (ExpansionHubMotor) hardwareMap.dcMotor.get("leftf");
        rightf = (ExpansionHubMotor) hardwareMap.dcMotor.get("rightf");

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

        bulkData = expansionHub.getBulkInputData();

        telemetry.addData("Left Rear", leftf.getVelocity());
        telemetry.addData("Right Rear", rightf.getVelocity());
        telemetry.addData("FlySwatter", ffRobot.flySwatter);

        //Drivetrain*******************************************************

        teleopDriver.tankDrive(gamepad1, TeleopDriver.Direction.BACKWARD);
        //Toggle Half Speed on the drivetrain
        if (gamepad1.left_trigger > 0) {
            // control the drive train at full speed
            teleopDriver.setMaxSpeed(.75f);
        } else if (gamepad1.right_trigger > 0) {
            teleopDriver.setMaxSpeed(.3f);
        } else {
            // control the drive train at 1/2 speed - Normal driving
            teleopDriver.setMaxSpeed(.6f);
        }

        if (joy1.toggle.a) {
            ffRobot.duckSpinner.setPower(.75);
        } else {
            ffRobot.duckSpinner.setPower(0);
        }

        // Arm
        if (gamepad2.y) {
            ffRobot.flySwatter.armHigh();
        } else if (gamepad2.b) {
            ffRobot.flySwatter.armPickup();
        } else if (Math.abs(gamepad2.right_stick_y) > 0.2) {
            ffRobot.flySwatter.arm(-gamepad2.right_stick_y);
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
    }
}

