package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaSkyStone;
import org.firstinspires.ftc.teamcode.RobotConfiguration.Skystone.SkystoneRobot;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.EncoderAutoDriver;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.VuforiaDriver;

import java.util.Arrays;
@Disabled
@Autonomous(group = "Blue" )
public class StoneFarBlue extends LinearOpMode {
    private VuforiaDriver vuforiaDriver;

    @Override
    public void runOpMode() {
        final SkystoneRobot skystoneRobot = new SkystoneRobot(this.hardwareMap);
        final EncoderAutoDriver encoderAutoDriver = new EncoderAutoDriver(skystoneRobot, this);
        skystoneRobot.initialize();

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
        encoderAutoDriver.spinLeft(-8,8);
        encoderAutoDriver.driveToDistance(-32);
        skystoneRobot.pickup1.reverse(-1);
        skystoneRobot.pickup2.reverse(-1);
        // Returning for another stone
        encoderAutoDriver.driveToDistance(48);
        encoderAutoDriver.spinRight(8, -8);
        skystoneRobot.pickup1.run(1);
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
        encoderAutoDriver.driveToDistance(12);
        */
    }
}
