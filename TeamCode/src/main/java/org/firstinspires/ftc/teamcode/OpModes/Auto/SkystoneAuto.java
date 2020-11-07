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

@Autonomous(group = "Blue" )
@Disabled
public class SkystoneAuto extends LinearOpMode {
    private VuforiaDriver vuforiaDriver;

    @Override
    public void runOpMode() {
        final SkystoneRobot skystoneRobot = new SkystoneRobot(this.hardwareMap);
        final EncoderAutoDriver encoderAutoDriver = new EncoderAutoDriver(skystoneRobot, this);
        skystoneRobot.initialize();

        vuforiaDriver = new VuforiaDriver(
                new VuforiaSkyStone(),
                Arrays.asList(VuforiaSkyStone.TRACKABLE_NAMES),
                "AUWvyFD/////AAABmaAT+Xe4Iks6qo/7ra3yndh8HtUQLcEHWOxW1ZN74TVmgwOIg6aPYTrI0zh6peSAzci+oaRE3SteoHRMP3gLxfzjW98ja2voVSq1M1wLw3jFhVyd6Yv/RzYQ+HWQi+rpDkyLUv7GKOj+vBmGwxW8oyrazADHJ0a0VNoNcX1vPZhN8bwN3xRUzlG0g35012sY2CehU613sQw7GNOqmCVAEfVHBrVN6EdcYWIj5i1YpsXWb899AigohjtDu83oO1MeZQCWTinNHHMr6KUCX+Dn1wGvbJBKJrWA5liHUBaUOUScEylDa6Zp5ulY6fWr2YUszVRiTwR8WeqoATNTcbarq6psjT/sbmeX42nE/MKkRA9F");
        vuforiaDriver.initialize(
                VuforiaLocalizer.CameraDirection.BACK,
                new Position(DistanceUnit.INCH, 4, 8, 0, 0),
                new Orientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, 90, -90, 0, 0));

        /*
        //Autonomous for Squatty the Robot Blue Side
        waitForStart();
        encoderAutoDriver.setPower(.5);
        skystoneRobot.platformServo.open();
        encoderAutoDriver.driveToDistance(10.5);
        // it worked at 11.5 a few times
        encoderAutoDriver.spinLeft(-7,7);
        skystoneRobot.getDrivetrain().setPowers(-.07, -.07);
        boolean hasTurned = false;
        while(skystoneRobot.getDrivetrain().getRightEncoderCount() > -800 && !hasTurned && opModeIsActive()) {
            if(vuforiaDriver.isVisible(VuforiaSkyStone.TRACKABLE_NAMES[0])) {

                hasTurned = true;
            }
            if (vuforiaDriver.getTrackableLocation(VuforiaSkyStone.TRACKABLE_NAMES[0]) != null) {
                encoderAutoDriver.spinRight(22, 22);
            }
        }
        encoderAutoDriver.setPower(.5);
        encoderAutoDriver.spinLeft(-8,8);
        skystoneRobot.releaseServo.open();
        skystoneRobot.pickup1.reverse(-0.99);
        skystoneRobot.pickup2.reverse(-0.99);
        encoderAutoDriver.driveToDistance(-32);
        encoderAutoDriver.driveToDistance(32);
        encoderAutoDriver.spinRight(8,-8);
        encoderAutoDriver.driveToDistance(70);
        encoderAutoDriver.spinLeft(-17,17);
        skystoneRobot.pickup1.run(0.99);
        skystoneRobot.pickup2.run(0.99);
        encoderAutoDriver.driveToDistance(300);
        */
    }
}
