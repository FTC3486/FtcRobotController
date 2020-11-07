package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotConfiguration.Skystone.SkystoneRobot;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.EncoderAutoDriver;

/*
    Filename: SkystoneFoundationAutoBlue.java

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
 *     -Created by Saatvik on 2/6/19.
 *     -

 */

@Autonomous (group = "Red" )
public class FoundationCloseRed extends LinearOpMode {
    @Override
    public void runOpMode() {
        final SkystoneRobot skystoneRobot = new SkystoneRobot(this.hardwareMap);
        final EncoderAutoDriver encoderAutoDriver = new EncoderAutoDriver(skystoneRobot, this);
        //RangeAutoDriver rangeAutoDriver = new RangeAutoDriver(rover, this);
        skystoneRobot.initialize();
        /*
        //Autonomous for Squatty the Robot Red Side - Foundation, Park Close
        waitForStart();
        encoderAutoDriver.setPower(.5);
        skystoneRobot.platformServo.open();
        skystoneRobot.armVertical.stop();
        encoderAutoDriver.driveToDistance(5);
        encoderAutoDriver.spinRight(2, -2);
        encoderAutoDriver.driveToDistance(5);
        encoderAutoDriver.spinLeft(-1.5, 1.5);
        encoderAutoDriver.driveToDistance(10
        );
        encoderAutoDriver.setPower(.15);
        encoderAutoDriver.spinRight(1.25,-1.25);
        encoderAutoDriver.driveToDistance(5.75);
        //encoderAutoDriver.spinRight(.25,-.25);
        skystoneRobot.platformServo.close();
        //sleep is meant to pause the robot for 1000 milliseconds
        sleep(2500);
        encoderAutoDriver.driveToDistance(1);
        encoderAutoDriver.driveToDistance(-1);
        encoderAutoDriver.setPower(.4);
        encoderAutoDriver.driveRightSideToDistancePower(-30);
        //encoderAutoDriver.driveToDistance(-15); //20
        //encoderAutoDriver.driveLeftSideToDistance(-20);// 9
        skystoneRobot.platformServo.open();
        sleep(2500);
        //encoderAutoDriver.driveToDistance(-2);
        encoderAutoDriver.setPower(.8);
        encoderAutoDriver.driveToDistance(-3);
        encoderAutoDriver.spinLeft(-14,14);
        //following was changed from 30 to 25
        encoderAutoDriver.driveToDistance(25);
        //skystoneRobot.releaseServo.open();
        sleep(3500);
        */
    }
}