package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotConfiguration.Skystone.SkystoneRobot;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.EncoderAutoDriver;

/*public class StoneCloseBlueTest
Created by Mary 2-20-20
Use this code
*/
@Disabled
@Autonomous (group = "Blue" )
public class StoneCloseBlueTest extends LinearOpMode {
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
        skystoneRobot.flapper.run(1);
        skystoneRobot.platformServo.open();
        //skystoneRobot.releaseServo.open();
        skystoneRobot.pickup1.reverse(-1);
        skystoneRobot.pickup2.reverse(-1);
        encoderAutoDriver.driveToDistance(-20);
        encoderAutoDriver.setPower(.5);
        encoderAutoDriver.driveToDistance(-10);
        skystoneRobot.releaseServo.open();
        //encoderAutoDriver.setPower(.5);
        sleep(1000);
        encoderAutoDriver.spinRight(6.6,-6.6);
        encoderAutoDriver.driveToDistance(-10);
        encoderAutoDriver.driveToDistance(10);
        sleep(500);
        skystoneRobot.releaseServo.close();
    // Stone collection
        /*skystoneRobot.pickup1.run(1);
        skystoneRobot.pickup2.run(1);
        encoderAutoDriver.driveToDistance(-15);
        encoderAutoDriver.setPower(.5);
        encoderAutoDriver.driveToDistance(-7);
        skystoneRobot.releaseServo.close();
        encoderAutoDriver.driveRightSideToDistancePower(-6);
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