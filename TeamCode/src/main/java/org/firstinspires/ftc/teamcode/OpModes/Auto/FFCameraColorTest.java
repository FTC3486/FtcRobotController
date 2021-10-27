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
@Autonomous(group = "Blue")
public class FFCameraColorTest extends LinearOpMode {

    OpenCvCamera webcam;
    SkystoneDeterminationPipeline pipeline;
    private UltimateGoalRobot ultimateGoalRobot;

    @Override
    public void runOpMode() {
        ultimateGoalRobot = new UltimateGoalRobot(this.hardwareMap);
        telemetry.addData("Test", ultimateGoalRobot.getDrivetrain().getLeftEncoderCount());
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
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }


        String comp = pipeline.state();

        // comp.equals("NONE")
        if (comp.equals("NONE")) {
            webcam.stopStreaming();

        } else if (comp.equals("ONE")) {
            webcam.stopStreaming();

        } else {
            webcam.stopStreaming();
        }

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

        final int FOUR_RING_THRESHOLD = 140;
        final int ONE_RING_THRESHOLD = 130;

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