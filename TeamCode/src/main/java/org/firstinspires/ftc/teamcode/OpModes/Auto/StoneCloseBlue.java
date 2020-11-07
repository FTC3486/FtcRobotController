package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotConfiguration.Skystone.SkystoneRobot;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.EncoderAutoDriver;

/*
    Filename: SkystoneCloseBlue.java

    Description:
        Test Autonomous program using encoders to drive Rover.

    Use:
        Test program to test encoderAutoDriver, JewelArm and RangeAutoDriver.  Test Stall monitor.

    Requirements:
 *     - AutoDrive configured for stall monitor
 *     - Drive motors with encoders
 *     - Two Range sensors
 *     - one color sensor
 *     -Jewel armVertical
 * *
 * Changelog:
     *     -Created by Saatvik on 2/6/20.
 *     -

 */
@Disabled
@Autonomous (group = "Blue" )
public class StoneCloseBlue extends LinearOpMode {
    @Override
    public void runOpMode() {
        final SkystoneRobot skystoneRobot = new SkystoneRobot(this.hardwareMap);
        final EncoderAutoDriver encoderAutoDriver = new EncoderAutoDriver(skystoneRobot, this);
        //RangeAutoDriver rangeAutoDriver = new RangeAutoDriver(rover, this);
        skystoneRobot.initialize();

        //Autonomous for Squatty the Robot Blue Side - Stone, Park Close
        waitForStart();
        /*
        encoderAutoDriver.setPower(1);
        skystoneRobot.platformServo.open();
        skystoneRobot.releaseServo.open();
        // Stone collection
        skystoneRobot.pickup1.run(1);
        skystoneRobot.pickup2.run(1);
        encoderAutoDriver.driveToDistance(-15);
        encoderAutoDriver.setPower(.1);
        encoderAutoDriver.driveToDistance(-15);
        encoderAutoDriver.setPower(1);
        encoderAutoDriver.driveToDistance(18);
        // Stone delivery
        //values for below can also be 6.5
        encoderAutoDriver.spinLeft(-6.6,6.6);
        encoderAutoDriver.driveToDistance(-32);
        skystoneRobot.pickup1.reverse(-1);
        skystoneRobot.pickup2.reverse(-1);
        // Returning for another stone
        encoderAutoDriver.driveToDistance(48);
        encoderAutoDriver.spinRight(6.6, -6.6);
        /*skystoneRobot.pickup1.run(1);
        skystoneRobot.pickup2.run(1);
        encoderAutoDriver.setPower(.1);
        encoderAutoDriver.driveToDistance(-15);
        encoderAutoDriver.driveToDistance(15);
        // Delivering the second stone
        encoderAutoDriver.setPower(1);
        encoderAutoDriver.spinLeft(-8, 8);
        encoderAutoDriver.driveToDistance(-52);
        skystoneRobot.pickup1.reverse(1);
        skystoneRobot.pickup2.reverse(1);
        //Parking on the line
        encoderAutoDriver.driveToDistance(12);*/
    }
}