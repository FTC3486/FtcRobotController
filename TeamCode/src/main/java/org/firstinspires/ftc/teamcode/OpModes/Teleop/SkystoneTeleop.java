package org.firstinspires.ftc.teamcode.OpModes.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotConfiguration.Skystone.SkystoneRobot;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.GamepadWrapper;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.TeleopDriver;


/**
 * Created by 3486 on 10/10/2019.
 */

@TeleOp(name = "Skystone Teleop", group = "Teleop2018")
public class SkystoneTeleop extends OpMode {
    //Declare parts of the robot that will be used by this Teleop
    private SkystoneRobot skystoneRobot;
    //Create joysticks
    private GamepadWrapper joy1 = new GamepadWrapper();
    private GamepadWrapper joy2 = new GamepadWrapper();
    private TeleopDriver teleopDriver;

    @Override
    public void init() {
        skystoneRobot = new SkystoneRobot(this.hardwareMap);
        skystoneRobot.initialize();
        teleopDriver = new TeleopDriver(skystoneRobot);
        teleopDriver.setMinSpeed(0.2);
        skystoneRobot.initialize();
        //skystoneRobot.releaseServo.open();
    }

    @Override
    public void loop() {
        //Gamepad 1 is the driver controller, gamepad 2 is the gunner controller
        joy1.update(gamepad1);
        joy2.update(gamepad2);

        //Drivetrain*******************************************************

        //Toggle Half Speed on the drivetrain
        if (gamepad1.left_trigger > 0) {
            // control the drive train at full speed
            teleopDriver.setMaxSpeed(1f);
        } else if (gamepad1.right_trigger>0){
            teleopDriver.setMaxSpeed(.3f);
        }

        else{
            // control the drive train at 1/2 speed - Normal driving
            teleopDriver.setMaxSpeed(.5f);
        }

        driveBackwardsToggle(joy1.toggle.back);

        //PICKUP Functions********************************************************
        //Pickup Release Servo


        if (joy1.toggle.a) {
            skystoneRobot.releaseServo.open();
        } else {
            skystoneRobot.releaseServo.close();
        }


        //Pickup Motor
        if (gamepad1.right_bumper) {
            skystoneRobot.pickup1.reverse(-0.99);
            skystoneRobot.pickup2.reverse(-0.99);
        } else if (gamepad1.y) {
            skystoneRobot.pickup1.run(.99);
            skystoneRobot.pickup2.run(.99);
        } else if (gamepad1.left_bumper) {
            skystoneRobot.pickup1.stop();
            skystoneRobot.pickup2.stop();
        }

        //Block Pusher
        if (joy2.toggle.a) {
            skystoneRobot.flapper.run(1);
        } else if(joy2.toggle.y){
            skystoneRobot.flapper.reverse(-1);
        } else {
            skystoneRobot.flapper.stop();
        }

        //ARM Functions**************************************************************

        //Buttons for the arm up/down
        if (gamepad2.right_stick_y > joy2.getLeftStickThreshold()) {
            skystoneRobot.armVertical.reverse(gamepad2.right_stick_y);
        } else if (gamepad2.right_stick_y < -joy2.getLeftStickThreshold()) {
            skystoneRobot.armVertical.run(gamepad2.right_stick_y);
        } else {
            skystoneRobot.armVertical.reverse(-.25);
            //skystoneRobot.armVertical.reverse(.5);
        }
        //Manipulator Functions **********************************************

        //Buttons for the Stone angle servo
        // Has 4 positions
        /*if (gamepad2.x) {
            skystoneRobot.angleServo.setPosition(POSITION_1);
        } else if (gamepad2.y) {
            skystoneRobot.angleServo.setPosition(POSITION_2);
        } else if (gamepad2.b) {
            skystoneRobot.angleServo.setPosition(POSITION_3);
        } else if (gamepad2.a) {
            skystoneRobot.angleServo.setPosition(POSITION_4);
        }*/

        //2nd joint 180 - rotates stone from in the armVertical to out - middleManipulatorServo
        if (joy2.toggle.left_bumper) {
            skystoneRobot.middleManipulatorServo.open();
        } else {
            skystoneRobot.middleManipulatorServo.close();
        }

        //Manipulator/grabber servo
        if (joy2.toggle.right_bumper) {
            skystoneRobot.manipulatorServo.close();
        } else {
            skystoneRobot.manipulatorServo.open();
        }

        //Capstone
        if (joy2.toggle.x) {
            skystoneRobot.capstoneServo.open();
        } else {
            skystoneRobot.capstoneServo.close();
        }


        //Platform
        if (joy1.toggle.b) {
            skystoneRobot.platformServo.open();
        } else {
            skystoneRobot.platformServo.close();
        }

    }

    private void driveBackwardsToggle(boolean toggle) {
        if (toggle) {
            teleopDriver.tankDrive(gamepad1, TeleopDriver.Direction.FORWARD);
        } else {
            teleopDriver.tankDrive(gamepad1, TeleopDriver.Direction.BACKWARD);
        }
    }
}