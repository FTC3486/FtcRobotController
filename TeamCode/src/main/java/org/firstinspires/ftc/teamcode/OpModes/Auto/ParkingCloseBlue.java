package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotConfiguration.Skystone.SkystoneRobot;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.EncoderAutoDriver;

/*
    Filename: ParkingCloseBlue.java

    Description:
        Park robot close to wall on Blue.

    Use:
        Parking robot close to the wall on Blue.

    Requirements:
 *     - AutoDrive configured for stall monitor
 *     - Drive motors with encoders
 *     - Two Range sensors
 *     - one color sensor
 *     -Jewel armVertical
 * *
 * Changelog:
     *     -Created by Saatvik on 2/27/20.

 */

@Autonomous (group = "Blue" )
public class ParkingCloseBlue extends LinearOpMode {
    @Override
    public void runOpMode() {
        final SkystoneRobot skystoneRobot = new SkystoneRobot(this.hardwareMap);
        final EncoderAutoDriver encoderAutoDriver = new EncoderAutoDriver(skystoneRobot, this);
        //RangeAutoDriver rangeAutoDriver = new RangeAutoDriver(rover, this);
        skystoneRobot.initialize();
        /*
        //Autonomous for Squatty the Robot Blue Side - Park Close
        waitForStart();

        double robotPower = 1;
        encoderAutoDriver.setPower(robotPower);

        double leftInches = -4;
        double rightInches = 4;
        encoderAutoDriver.spinLeft(leftInches, rightInches);
        sleep(1000);
        double distance = 13;
        encoderAutoDriver.driveToDistance(distance);
        sleep(1000);
        encoderAutoDriver.driveToDistance(-distance);
        sleep(1000);
        encoderAutoDriver.driveLeftSideToDistance(13);
        sleep(1000);
        skystoneRobot.releaseServo.open();
        sleep(1000);


        skystoneRobot.releaseServo.close();

         */
    }
}