package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotConfiguration.Skystone.SkystoneRobot;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.EncoderAutoDriver;

@Autonomous(group = "Red" )
@Disabled
public class SkystoneFoundationAutoRed extends LinearOpMode {
    @Override
    public void runOpMode() {
        final SkystoneRobot skystoneRobot = new SkystoneRobot(this.hardwareMap);
        final EncoderAutoDriver encoderAutoDriver = new EncoderAutoDriver(skystoneRobot, this);
        //RangeAutoDriver rangeAutoDriver = new RangeAutoDriver(rover, this);
        skystoneRobot.initialize();

        //Autonomous for Squatty the Robot Blue Side - Foundation
        waitForStart();
        /*
        encoderAutoDriver.setPower(0.4);
        skystoneRobot.platformServo.open();
        encoderAutoDriver.driveToDistance(15);
        encoderAutoDriver.spinRight(.8, -.8);
        encoderAutoDriver.driveToDistance(20);
        encoderAutoDriver.spinRight(9.5,-9.5);
        encoderAutoDriver.setPower(.15);
        encoderAutoDriver.driveToDistance(4.5);
        skystoneRobot.platformServo.close();
        sleep(2500);
        encoderAutoDriver.driveToDistance(1);
        encoderAutoDriver.setPower(.6);
        encoderAutoDriver.driveToDistance(-3);
        encoderAutoDriver.spinRight(6, -6);
        encoderAutoDriver.driveToDistance(16);
        //skystoneRobot.platformServo.open();
        sleep(2500);
        encoderAutoDriver.driveToDistance(-4);
        encoderAutoDriver.spinRight(7, -7);
        encoderAutoDriver.driveToDistance(30);
        /*encoderAutoDriver.driveToDistance(5);
        encoderAutoDriver.spinRight(2, -2);
        encoderAutoDriver.driveToDistance(15);
        encoderAutoDriver.spinLeft(-2, 2);
        encoderAutoDriver.setPower(.15);
        encoderAutoDriver.spinRight(1, -1);
        encoderAutoDriver.driveToDistance(7);
        //encoderAutoDriver.spinRight(.25,-.25);
        skystoneRobot.platformServo.close();
        //sleep is meant to pause the robot for 1000 milliseconds
        sleep(2500);
        encoderAutoDriver.setPower(.4);
        encoderAutoDriver.driveToDistance(-22); //20
        encoderAutoDriver.driveRightSideToDistance(-4);// 9
        skystoneRobot.platformServo.open();
        sleep(2500);
        encoderAutoDriver.driveToDistance(-2);
        encoderAutoDriver.setPower(.8);
        encoderAutoDriver.spinRight(7, -7);
        encoderAutoDriver.driveToDistance(-30);
        //skystoneRobot.releaseServo.open();
        sleep(3500);*/

    }
}
