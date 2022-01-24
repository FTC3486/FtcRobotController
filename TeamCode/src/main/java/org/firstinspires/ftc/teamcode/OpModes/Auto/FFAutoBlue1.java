package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.RobotConfiguration.FreightFrenzy.FreightFrenzyRobot;
import org.firstinspires.ftc.teamcode.RobotConfiguration.UltimateGoal.UltimateGoalRobot;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.EncoderAutoDriver;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.TeleopDriver;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.LinkedList;

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
 *     -Created by Saatvik on 12/22/20.
 *     -
 */
@Disabled
@Autonomous (group = "Blue" )
public class FFAutoBlue1 extends LinearOpMode {

    BNO055IMU               imu;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    boolean top_wobble_goal = false;
    boolean mid_wobble_goal = false;
    boolean bottom_wobble_goal = false;

    @Override
    public void runOpMode() {


        final FreightFrenzyRobot ffRobot = new FreightFrenzyRobot(this.hardwareMap);
        final EncoderAutoDriver encoderAutoDriver = new EncoderAutoDriver(ffRobot, this);
        ExpansionHubEx expansionHub;
        RevBulkData bulkData;
        ExpansionHubMotor leftf, leftr, rightf, rightr;
        //RangeAutoDriver rangeAutoDriver = new RangeAutoDriver(rover, this);
        ffRobot.getDrivetrain().resetMotorEncoders();
        ffRobot.initialize();
        //expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        //bulkData = expansionHub.getBulkInputData();
        //expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        //leftf = (ExpansionHubMotor) hardwareMap.dcMotor.get("leftf");
        leftr = (ExpansionHubMotor) hardwareMap.dcMotor.get("leftr");

        waitForStart();
        encoderAutoDriver.setPower(1);

        boolean leftActive = false;
        boolean rightActive = false;

        //initialize imu
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        encoderAutoDriver.setImu(imu);
        imu.initialize(parameters);
        /*
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }
        */

        //ffRobot.getDrivetrain().setPowers(-.3, -.3);

        //CHECK FOR TEAM MARKER
        //left = 0, middle = 1, right = 2
       // int winning_sensor = 0;
     //   int[] sensor_detections = {0,0,0};

        //left sensor means bottom wobble goal
        //mid sensor means mid wobble goal
        //right sensor mean top wobble goal
 //(ffRobot.distance_right() < 15) {
               // sensor_detections[2]++;
   //         }
    //    }
/*
        for(int i = 0; i < 50; i++) {
            if (ffRobot.distance_left() < 15) {
                sensor_detections[0]++;
            } else if (ffRobot.distance_middle() < 15) {
                sensor_detections[1]++;
            } else if
        //determine winning sensor
        for(int i = 0; i < sensor_detections.length; i++){
            if(sensor_detections[winning_sensor] < sensor_detections[i]){
                winning_sensor = i;
            }
        }


        //DO MOVEMENT

        //move forward a little so rotation doesn't hit wall
        encoderAutoDriver.driveToDistance(3, 0.5);

        //rotate 90deg to face spinner
        encoderAutoDriver.rotateClockwise(90, hardwareMap);

        //move to spinner
        encoderAutoDriver.driveToDistance(6, 0.5);

        //move backwards towards the wobble goal
        encoderAutoDriver.driveToDistance(-35, 0.5);

        //rotate 90deg to wobble goal
        encoderAutoDriver.rotateCounterClockwise(90, hardwareMap);

        //move backwards towards the wobble goal
        encoderAutoDriver.driveToDistance(10, 0.5);

        //move backwards towards the wobble goal
        encoderAutoDriver.driveToDistance(-10, 0.5);

        //rotate 90deg to wobble goal
        encoderAutoDriver.rotateCounterClockwise(90, hardwareMap);

        //move backwards towards the wobble goal
        encoderAutoDriver.driveToDistance(10, 0.5);

        telemetry.update();
        sleep(5000);
*/

    }
}
