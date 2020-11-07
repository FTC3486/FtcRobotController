package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotConfiguration.Skystone.SkystoneRobot;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.EncoderAutoDriver;

/*
    Filename: RoverDepotAuto.java

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
 *     -Created by 3486 on 7/5/18.
 *     -

 */

@Autonomous (group = "Blue" )
public class RoverDepotAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        SkystoneRobot skystoneRobot = new SkystoneRobot(this.hardwareMap);
        EncoderAutoDriver encoderAutoDriver = new EncoderAutoDriver(skystoneRobot, this);
        //RangeAutoDriver rangeAutoDriver = new RangeAutoDriver(rover, this);
        skystoneRobot.initialize();

        waitForStart();

        encoderAutoDriver.setPower(1.0);

        while(!skystoneRobot.latch.isFullyExtended())
        {
            skystoneRobot.latch.extend();
            telemetry.addData("Latch state", skystoneRobot.latch.toString());
            telemetry.update();
        }

        //This is copy and pasted from RoverDepotAuto.java, the original code is commented out
        skystoneRobot.latch.manualStop();
        encoderAutoDriver.setPower(0.75);
        //base number 25
        encoderAutoDriver.driveLeftSideToDistance(-25);
        encoderAutoDriver.driveToDistance(-7);
        //larger numbers turn to blocks - right turn
        encoderAutoDriver.spinRight(10,-10);
        encoderAutoDriver.driveToDistance(-8);
        encoderAutoDriver.spinRight(0.4, -0.4);

        //Go to the end of the sampling items
        skystoneRobot.getDrivetrain().resetMotorEncoders();
        skystoneRobot.getDrivetrain().setPowers(0.3, 0.3);
        //while ((skystoneRobot.getDrivetrain().getLeftEncoderCount() <= 2700) && !skystoneRobot.foundYellowObject() && opModeIsActive())
       // {
            //telemetry.addData("Green Value", skystoneRobot.colorSensor.green());
            //telemetry.addData("Blue Value", skystoneRobot.colorSensor.blue());
            //telemetry.addData("Red Value", skystoneRobot.colorSensor.red());
            //telemetry.addData("LeftEncoder", skystoneRobot.getDrivetrain().getLeftEncoderCount());
            //telemetry.update();
       // }
        double counts = skystoneRobot.getDrivetrain().getLeftEncoderCount();
        skystoneRobot.getDrivetrain().haltDrive();
        encoderAutoDriver.spinLeft(-10,10);
        encoderAutoDriver.driveToDistance(-6);
        encoderAutoDriver.driveToDistance(6);
        encoderAutoDriver.spinRight(10, -10);
        skystoneRobot.getDrivetrain().setPowers(0.5, 0.5);
       // while(skystoneRobot.getDrivetrain().getLeftEncoderCount() <= 4825 - counts && opModeIsActive()) {telemetry.addData("Encoder", skystoneRobot.getDrivetrain().getLeftEncoderCount());}
        encoderAutoDriver.spinLeft(-7.7, 7.7);
        encoderAutoDriver.driveToDistance(-50);
        //skystoneRobot.markerServo.open();
        encoderAutoDriver.driveToDistance(1.0);
    }
}