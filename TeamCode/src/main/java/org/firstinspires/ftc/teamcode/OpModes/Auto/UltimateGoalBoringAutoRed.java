package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareDeviceCloseOnTearDown;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotConfiguration.UltimateGoal.UltimateGoalRobot;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.EncoderAutoDriver;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

/*
    Filename: UltimateGoalAutoBlue.java
    Description:
        Autonomous program for the Ultimate Goal season on the Blue team.
    Use:
        Deliver Wobble Goal and Park
    Requirements:
 *     - AutoDrive configured for stall monitor
 *     - Drive motors with encoders
 *     - Two Range sensors
 *     - one color sensor
 *     -Jewel armVertical
 * *
 * Changelog:
 *     -Created by Saatvik on 12/27/20.
 *     -Copy and pasted from the original on 6/5/21
       -Edited by Mary on 6/24/21
 */

@Autonomous(name = "Ultimate Goal Boring Auto Red", group = "Blue", preselectTeleOp = "UltimateGoalTeleop")
public class UltimateGoalBoringAutoRed extends LinearOpMode {

    private UltimateGoalRobot ultimateGoalRobot;


    @Override
    public void runOpMode() {
        ultimateGoalRobot = new UltimateGoalRobot(this.hardwareMap);
        final EncoderAutoDriver encoderAutoDriver = new EncoderAutoDriver(ultimateGoalRobot, this);
        telemetry.addData("Test", ultimateGoalRobot.getDrivetrain().getLeftEncoderCount());
        //RangeAutoDriver rangeAutoDriver = new RangeAutoDriver(rover, this);
        ultimateGoalRobot.getDrivetrain().resetMotorEncoders();
        ultimateGoalRobot.initialize();
        ultimateGoalRobot.armGripServo.close();

        ExpansionHubEx expansionHub;
        RevBulkData bulkData;
        ExpansionHubMotor leftf, leftr, rightf, rightr;
        //RangeAutoDriver rangeAutoDriver = new RangeAutoDriver(rover, this);
        ultimateGoalRobot.getDrivetrain().resetMotorEncoders();
        ultimateGoalRobot.initialize();
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        bulkData = expansionHub.getBulkInputData();
        leftf = (ExpansionHubMotor) hardwareMap.dcMotor.get("leftf");
        leftr = (ExpansionHubMotor) hardwareMap.dcMotor.get("leftr");

        telemetry.update();

        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using a webcam. Note that you will need to
         * make sure you have added the webcam to your configuration file and
         * adjusted the name here to match what you named it in said config file.
         *
         * We pass it the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.


        telemetry.update();

        //wobble goal drop
        waitForStart();
        ultimateGoalRobot.armGripServo.close();
        telemetry.addData("Left Encoder", ultimateGoalRobot.getDrivetrain().getLeftEncoderCount());
        telemetry.addData("Right Encoder", ultimateGoalRobot.getDrivetrain().getRightEncoderCount());

        encoderAutoDriver.setPower(1);

        //color sensors: will be false until they become true
        encoderAutoDriver.driveToDistance(-64, 1);
        encoderAutoDriver.driveToDistance(10, 1);
        boolean leftActive = false;
        boolean rightActive = false;
        ultimateGoalRobot.getDrivetrain().setPowers(-.3, -.3);
        while (!leftActive ||
                !rightActive) {
            //telemetry.addData sends data to be printed on the phone screen
            telemetry.addData("Left Color Sensor", ultimateGoalRobot.colorSensorLeft.green());
            telemetry.addData("Right Color Sensor", ultimateGoalRobot.colorSensorRight.green());
            telemetry.addData("Left Sensor", leftActive);
            telemetry.addData("Right Sensor", rightActive);
            updateTelemetry(telemetry);
            if (ultimateGoalRobot.colorSensorLeft.green() > 300) {
                leftActive = true;
            } if (ultimateGoalRobot.colorSensorRight.green() > 300) {
                rightActive = true;
            }

            if (leftActive) {
                ultimateGoalRobot.getDrivetrain().setPowers(-.3, 0);
            } else if(rightActive) {
                ultimateGoalRobot.getDrivetrain().setPowers(0, -.3);
            }
        }
        ultimateGoalRobot.getDrivetrain().setPowers(0, 0);
        encoderAutoDriver.driveToDistance(2, 1);
        ultimateGoalRobot.armAngleServo.open();
        ultimateGoalRobot.armGripServo.open();

        ultimateGoalRobot.delivery1.setVelocity(-2375);
        ultimateGoalRobot.delivery2.setVelocity(2375);
        sleep(1000);
        throwRings();
        encoderAutoDriver.driveToDistance(-6, 1);
        encoderAutoDriver.turnLeft(6,-.5,.5);
        encoderAutoDriver.driveToDistance(-6,.5);
        encoderAutoDriver.turnRight(6, .5,-.5);
    }

    //throwing rings
    public void throwRings() {
        ultimateGoalRobot.belt.reverse(-0.99);
        ultimateGoalRobot.ringServo.setPower(-0.99);

        sleep(5000);
        ultimateGoalRobot.belt.stop();
        ultimateGoalRobot.ringServo.setPower(0);
        ultimateGoalRobot.delivery1.stop();
        ultimateGoalRobot.delivery2.stop();
    }

}