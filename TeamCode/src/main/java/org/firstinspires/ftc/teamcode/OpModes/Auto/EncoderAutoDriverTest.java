package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotConfiguration.Skystone.SkystoneRobot;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.EncoderAutoDriver;

@Autonomous (group = "Blue" )
@Disabled
public class EncoderAutoDriverTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        final SkystoneRobot skystoneRobot = new SkystoneRobot(this.hardwareMap);
        final EncoderAutoDriver encoderAutoDriver = new EncoderAutoDriver(skystoneRobot, this);

        //RangeAutoDriver rangeAutoDriver = new RangeAutoDriver(rover, this);
        skystoneRobot.initialize();

        //Autonomous for Squatty the Robot Blue Side - Park Close
        waitForStart();

        encoderAutoDriver.driveToDistance(5, 0.5);
        sleep(5000);

        encoderAutoDriver.driveToDistance(-5, 0.5);
        sleep(5000);

        encoderAutoDriver.turnLeft(5, 0.5, 1.0);
        sleep(5000);

        encoderAutoDriver.spinCounterclockwise(5, 1.0);
        sleep(5000);

        encoderAutoDriver.turnRight(5, 1.0, 0.5);
        sleep(5000);

        encoderAutoDriver.spinClockwise(5, 1.0);
        sleep(5000);
    }
}
