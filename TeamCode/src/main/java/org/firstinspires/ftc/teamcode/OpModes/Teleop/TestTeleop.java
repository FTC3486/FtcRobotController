package org.firstinspires.ftc.teamcode.OpModes.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotConfiguration.Test.Test;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.GamepadWrapper;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.TeleopDriver;


/**
 * Created by 3486 on 10/10/2019.
 */

@TeleOp(name = "Test Teleop", group = "Teleop2018")
public class TestTeleop extends OpMode {
    //Declare parts of the robot that will be used by this Teleop
    private Test test;
    //Create joysticks
    private GamepadWrapper joy1 = new GamepadWrapper();
    private GamepadWrapper joy2 = new GamepadWrapper();
    private TeleopDriver teleopDriver;

    @Override
    public void init() {
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
        } else if (gamepad1.right_trigger > 0){
            teleopDriver.setMaxSpeed(.3f);
        }

        else{
            // control the drive train at 1/2 speed - Normal driving
            teleopDriver.setMaxSpeed(.5f);
        }

        driveBackwardsToggle(joy1.toggle.back);

    }

    private void driveBackwardsToggle(boolean toggle) {
        if (toggle) {
            teleopDriver.tankDrive(gamepad1, TeleopDriver.Direction.FORWARD);
        } else {
            teleopDriver.tankDrive(gamepad1, TeleopDriver.Direction.BACKWARD);
        }
    }
}