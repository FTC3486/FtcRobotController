package org.firstinspires.ftc.teamcode.OpModes.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotConfiguration.UltimateGoal.LordOfTheRingsRobot;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.GamepadWrapper;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.TeleopDriver;

@TeleOp(name = "Lord of the Rings", group = "Teleop")
public class LordOfTheRings extends OpMode {
    private LordOfTheRingsRobot lordOfTheRingsRobot;
    //Create robot driver
    private TeleopDriver teleopDriver;

    @Override
    public void init() {
        lordOfTheRingsRobot = new LordOfTheRingsRobot(this.hardwareMap);
        teleopDriver = new TeleopDriver(lordOfTheRingsRobot);
        teleopDriver.setMaxSpeed(0.5);
        teleopDriver.setMinSpeed(0.1);
        lordOfTheRingsRobot.initialize();
    }

    @Override
    public void loop() {
        teleopDriver.tankDrive(gamepad1, TeleopDriver.Direction.BACKWARD);

        if (gamepad1.left_bumper) {
            lordOfTheRingsRobot.tapeMeasure.tiltUp();
        } else if (gamepad1.left_trigger > 0.5) {
            lordOfTheRingsRobot.tapeMeasure.tiltDown();
        } else {
            lordOfTheRingsRobot.tapeMeasure.stopTilt();
        }

        if (gamepad1.right_bumper) {
            lordOfTheRingsRobot.tapeMeasure.retractTapeMeasure();
        } else if (gamepad1.right_trigger > 0.5) {
            lordOfTheRingsRobot.tapeMeasure.extendTapeMeasure();
        } else {
            lordOfTheRingsRobot.tapeMeasure.stopTapeMeasure();
        }

        telemetry.addData("TapeMeasure", lordOfTheRingsRobot.tapeMeasure);
        telemetry.update();
    }
}
