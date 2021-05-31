package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
 *     -
 */
@Autonomous(name = "Ultimate Goal Auto Blue", group = "Blue", preselectTeleOp = "UltimateGoalTeleop")
public class UltimateGoalAutoBlue extends LinearOpMode {

    OpenCvCamera webcam;
    SkystoneDeterminationPipeline pipeline;
    private UltimateGoalRobot ultimateGoalRobot;

    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;
    boolean                 aButton, bButton, touched;

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
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

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
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

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        telemetry.addData("Analysis", pipeline.getAnalysis());
        telemetry.addData("Position", pipeline.state());
        telemetry.update();

        waitForStart();
        ultimateGoalRobot.armGripServo.close();
        for (int i = 0; i < 40; i++) {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.state());
            telemetry.addData("Left Encoder", ultimateGoalRobot.getDrivetrain().getLeftEncoderCount());
            telemetry.addData("Right Encoder", ultimateGoalRobot.getDrivetrain().getRightEncoderCount());
            telemetry.addData("IMU", getAngle());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
        telemetry.addData("Left Encoder", ultimateGoalRobot.getDrivetrain().getLeftEncoderCount());
        telemetry.addData("Right Encoder", ultimateGoalRobot.getDrivetrain().getRightEncoderCount());
        telemetry.addData("IMU", getAngle());
        updateTelemetry(telemetry);
        encoderAutoDriver.setPower(1);
        String comp = pipeline.state();
        resetAngle();
        // comp.equals("NONE")
        if (comp.equals("NONE")) {
            //encoderAutoDriver.driveToDistance(100, .5);
            webcam.stopStreaming();
            // Originally at .93, I presume better in the 13.3-13.8 volt range.
            //encoderAutoDriver.coast(12, .5, .5);
            //Originally 50, 40 with coast
            encoderAutoDriver.driveToDistance(50, 1);
            telemetry.addData("Left Encoder", ultimateGoalRobot.getDrivetrain().getLeftEncoderCount());
            telemetry.addData("Right Encoder", ultimateGoalRobot.getDrivetrain().getRightEncoderCount());
            updateTelemetry(telemetry);
            ultimateGoalRobot.armAngleServo.open();
            sleep(500);
            ultimateGoalRobot.armGripServo.open();
            sleep(500);
            encoderAutoDriver.coast(-2, -.5, -.5);
            encoderAutoDriver.driveToDistance(-10, 1);
            telemetry.addData("IMU", getAngle());
            updateTelemetry(telemetry);
            ultimateGoalRobot.armAngleServo.close();
            ultimateGoalRobot.armGripServo.close();
            sleep(500);
            telemetry.addData("IMU", getAngle());
            updateTelemetry(telemetry);
            encoderAutoDriver.spinCounterclockwise(7.5, .5);
            ultimateGoalRobot.getDrivetrain().setPowers(.5, .5);
            sleep(1000);
            ultimateGoalRobot.getDrivetrain().setPowers(0, 0);
            //22.5 for all paths
            encoderAutoDriver.driveToDistance(-20, 1);
            encoderAutoDriver.spinCounterclockwise(7.5, .5);
            encoderAutoDriver.driveToDistance(-6, 1);
            telemetry.addData("IMU", getAngle());
            updateTelemetry(telemetry);
            //comp.equals("ONE")
        } else if (comp.equals("ONE")) {
            webcam.stopStreaming();
            //encoderAutoDriver.coast(12, .5, .5);
            // 60 with coast
            encoderAutoDriver.driveToDistance(75, 1);
            telemetry.addData("Left Encoder", ultimateGoalRobot.getDrivetrain().getLeftEncoderCount());
            telemetry.addData("Right Encoder", ultimateGoalRobot.getDrivetrain().getRightEncoderCount());
            telemetry.update();
            encoderAutoDriver.spinClockwise(6.5, .5);
            encoderAutoDriver.driveToDistance(8, 1);
            ultimateGoalRobot.armAngleServo.open();
            sleep(500);
            ultimateGoalRobot.armGripServo.open();
            sleep(500);
            encoderAutoDriver.driveToDistance(-8, 1);
            ultimateGoalRobot.armAngleServo.close();
            ultimateGoalRobot.armGripServo.close();
            sleep(500);
            encoderAutoDriver.spinCounterclockwise(6.5, .5);
            encoderAutoDriver.driveToDistance(-35, 1);
            telemetry.update();
            encoderAutoDriver.spinCounterclockwise(7.5, .5);
            ultimateGoalRobot.getDrivetrain().setPowers(.5, .5);
            sleep(1000);
            ultimateGoalRobot.getDrivetrain().setPowers(0, 0);
            encoderAutoDriver.driveToDistance(-20, 1);
            encoderAutoDriver.spinCounterclockwise(7.5, .5);
            encoderAutoDriver.driveToDistance(-6, 1);
        } else {
            webcam.stopStreaming();
            //encoderAutoDriver.coast(12, .5, .5);
            // Originally 90
            encoderAutoDriver.turnLeft(100, 1, 1);
            telemetry.addData("Left Encoder", ultimateGoalRobot.getDrivetrain().getLeftEncoderCount());
            telemetry.addData("Right Encoder", ultimateGoalRobot.getDrivetrain().getRightEncoderCount());
            telemetry.update();
            ultimateGoalRobot.armAngleServo.open();
            sleep(500);
            ultimateGoalRobot.armGripServo.open();
            sleep(500);
            encoderAutoDriver.driveToDistance(-6, 1);
            ultimateGoalRobot.armAngleServo.close();
            ultimateGoalRobot.armGripServo.close();
            sleep(500);
            encoderAutoDriver.coast(-12, -.5, -.5);
            encoderAutoDriver.driveToDistance(-36, 1);
            telemetry.update();
            encoderAutoDriver.spinCounterclockwise(7.5, .5);
            ultimateGoalRobot.getDrivetrain().setPowers(.5, .5);
            sleep(1000);
            ultimateGoalRobot.getDrivetrain().setPowers(0, 0);
            encoderAutoDriver.driveToDistance(-20, 1);
            encoderAutoDriver.spinCounterclockwise(7.5, .5);
            encoderAutoDriver.driveToDistance(-6, 1);
        }

        telemetry.addData("IMU", getAngle());
        updateTelemetry(telemetry);

        while ((getAngle() > -178 && getAngle() < 0) || (getAngle() > 182 && getAngle() > 0)) {
            ultimateGoalRobot.getDrivetrain().setPowers(.3, 0);
        }
        ultimateGoalRobot.getDrivetrain().setPowers(0, 0);
        
        while ((getAngle() < 178 && getAngle() > 0) || (getAngle() < -182 && getAngle() < 0) ) {
            ultimateGoalRobot.getDrivetrain().setPowers(0, .3);
        }
        ultimateGoalRobot.getDrivetrain().setPowers(0, 0);
        boolean leftActive = false;
        boolean rightActive = false;
        ultimateGoalRobot.getDrivetrain().setPowers(-.3, -.3);
        while (!leftActive ||
                !rightActive) {
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
        /*
        encoderAutoDriver.driveToDistance(3, .3);
        sleep(500);
        leftActive = false;
        rightActive = false;
        ultimateGoalRobot.getDrivetrain().setPowers(-.3, -.3);
        while (!leftActive ||
                !rightActive) {
            telemetry.addData("Left Color Sensor", ultimateGoalRobot.colorSensorLeft.green());
            telemetry.addData("Right Color Sensor", ultimateGoalRobot.colorSensorRight.green());
            telemetry.addData("Left Sensor", leftActive);
            telemetry.addData("Right Sensor", rightActive);
            updateTelemetry(telemetry);
            if (ultimateGoalRobot.colorSensorLeft.green() > 500) {
                leftActive = true;
            } if (ultimateGoalRobot.colorSensorRight.green() > 500) {
                rightActive = true;
            }

            if (leftActive) {
                ultimateGoalRobot.getDrivetrain().setPowers(-.3, 0);
            } else if(rightActive) {
                ultimateGoalRobot.getDrivetrain().setPowers(0, -.3);
            }
        }
        ultimateGoalRobot.getDrivetrain().setPowers(0, 0);

         */
        encoderAutoDriver.driveToDistance(2, .5);
        telemetry.addData("IMU", getAngle());
        updateTelemetry(telemetry);
        ultimateGoalRobot.delivery1.setVelocity(-2300);
        ultimateGoalRobot.delivery2.setVelocity(2300);
        sleep(1000);
        throwRings();
        encoderAutoDriver.driveToDistance(-6, 1);
    }

    public void throwRings() {
        ultimateGoalRobot.belt.reverse(-0.99);
        ultimateGoalRobot.ringServo.setPower(-0.99);

        sleep(5000);
        ultimateGoalRobot.belt.stop();
        ultimateGoalRobot.ringServo.setPower(0);
        ultimateGoalRobot.delivery1.stop();
        ultimateGoalRobot.delivery2.stop();
    }
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline {

        /*
         * An enum to define the skystone position
         */
        public enum RingPosition {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        // Original: 181, 98
        // Increasing x moves point right
        // Increasing y moves point down
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(270, 110);

        static final int REGION_WIDTH = 45;
        static final int REGION_HEIGHT = 40;

        final int FOUR_RING_THRESHOLD = 150;
        final int ONE_RING_THRESHOLD = 137;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile RingPosition position = RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingPosition.FOUR; // Record our analysis
            if (avg1 > FOUR_RING_THRESHOLD) {
                position = RingPosition.FOUR;
            } else if (avg1 > ONE_RING_THRESHOLD) {
                position = RingPosition.ONE;
            } else {
                position = RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis() {
            return avg1;
        }

        public String state() {
            if (position == RingPosition.NONE) {
                return "NONE";
            } else if (position == RingPosition.ONE) {
                return "ONE";
            } else if (position == RingPosition.FOUR) {
                return "FOUR";
            } else {
                return "ERROR";
            }
        }
    }
}