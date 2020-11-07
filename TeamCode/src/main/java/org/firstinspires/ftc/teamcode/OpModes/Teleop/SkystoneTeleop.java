package org.firstinspires.ftc.teamcode.OpModes.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotConfiguration.Skystone.SkystoneRobot;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.GamepadWrapper;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.TeleopDriver;

import static org.firstinspires.ftc.teamcode.RobotConfiguration.Skystone.SkystoneRobot.AngleServoPosition.*;

/**
 * Created by 3486 on 7/15/2017.
 */

@TeleOp(name = "Rover Teleop", group = "Teleop2018")
public class SkystoneTeleop extends OpMode {
    //Declare parts of the robot that will be used by this Teleop
    private SkystoneRobot skystoneRobot;
    //
    private GamepadWrapper joy1 = new GamepadWrapper();
    private GamepadWrapper joy2 = new GamepadWrapper();
    private TeleopDriver teleopDriver;

    @Override
    public void init() {
        skystoneRobot = new SkystoneRobot(this.hardwareMap);
        skystoneRobot.initialize();
        teleopDriver = new TeleopDriver(skystoneRobot);
        skystoneRobot.initialize();
        //skystoneRobot.jewelArm.fullyExtend();
    }

    @Override
    public void loop() {
        //Gamepad 1 is the driver controller, gamepad 2 is the gunner controller
        joy1.update(gamepad1);
        joy2.update(gamepad2);

        //Drivetrain*******************************************************

        //Toggle Half Speed on the drivetrain
        if (joy1.toggle.right_stick_button) {
            // control the drive train at half speed
            teleopDriver.setMaxSpeed(1f);
        } else {
            // control the drive train
            teleopDriver.setMaxSpeed(.5f);
        }

        driveBackwardsToggle(joy1.toggle.left_stick_button);

        //PICKUP Functions********************************************************

        //Pickup Release Servo
        if (gamepad1.a) {
            skystoneRobot.releaseServo.open();
        } else if (gamepad1.b) {
            skystoneRobot.releaseServo.close();
        }

        //Pickup Motor
        if (gamepad1.right_bumper) {
            skystoneRobot.pickup1.run(.9);
            skystoneRobot.pickup2.run(.9);
        } else if (gamepad1.left_bumper) {
            skystoneRobot.pickup1.reverse(-0.5);
            skystoneRobot.pickup2.reverse(-0.5);
        } else {
            skystoneRobot.pickup1.stop();
            skystoneRobot.pickup2.stop();
        }

        //ARM Functions**************************************************************

        //Buttons for the arm extension/retraction
        if (gamepad2.left_stick_y > joy2.getLeftStickThreshold()) {
            skystoneRobot.armHorizontal.reverse(gamepad2.left_stick_y);
        } else if (gamepad2.left_stick_y < -joy2.getLeftStickThreshold()) {
            skystoneRobot.armHorizontal.run(gamepad2.left_stick_y);
        } else {
            skystoneRobot.armHorizontal.stop();
        }
        //Buttons for the arm up/down
        if (gamepad2.right_stick_y > joy2.getLeftStickThreshold()) {
            skystoneRobot.armVertical.reverse(gamepad2.right_stick_y);
        } else if (gamepad2.right_stick_y < -joy2.getLeftStickThreshold()) {
            skystoneRobot.armVertical.run(gamepad2.right_stick_y);
        } else {
            skystoneRobot.armVertical.stop();
        }
        //Manipulator Functions **********************************************

        //Buttons for the Stone angle servo
        // Has 4 positions

        if (gamepad2.x) {
            skystoneRobot.angleServo.setPosition(POSITION_1);
        } else if (gamepad2.y) {
            skystoneRobot.angleServo.setPosition(POSITION_2);
        } else if (gamepad2.b) {
            skystoneRobot.angleServo.setPosition(POSITION_3);
        } else if (gamepad2.a) {
            skystoneRobot.angleServo.setPosition(POSITION_4);
        }

        //2nd joint 180 - rotates stone from in the armVertical to out - middleManipulatorServo
        if (joy2.toggle.left_bumper) {
            skystoneRobot.middleManipulatorServo.open();
        } else {
            skystoneRobot.middleManipulatorServo.close();
        }
        
        //Manipulator/grabber servo
        if (joy2.toggle.right_bumper) {
            skystoneRobot.manipulatorServo.open();
        } else {
            skystoneRobot.manipulatorServo.close();
        }

        //Capstone
        if (gamepad1.x) {
            skystoneRobot.capstoneServo.open();
        } else {
            skystoneRobot.capstoneServo.close();
        }

        //Skystone**********************************************

        //Skystone server
    }

    private void driveBackwardsToggle(boolean toggle) {
        if (toggle) {
            teleopDriver.tankDrive(gamepad1, TeleopDriver.Direction.BACKWARD);
        } else {
            teleopDriver.tankDrive(gamepad1, TeleopDriver.Direction.FORWARD);
        }
    }
}
