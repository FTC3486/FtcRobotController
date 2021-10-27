package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotConfiguration.UltimateGoal.UltimateGoalRobot;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.EncoderAutoDriver;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.TeleopDriver;
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
 *     -Created by Saatvik on 12/22/20.
 *     -
 */

@Autonomous (group = "Blue" )
public class FFAutoBlue1 extends LinearOpMode {

    BNO055IMU               imu;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    @Override
    public void runOpMode() {


        final UltimateGoalRobot ultimateGoalRobot = new UltimateGoalRobot(this.hardwareMap);
        final EncoderAutoDriver encoderAutoDriver = new EncoderAutoDriver(ultimateGoalRobot, this);
        ExpansionHubEx expansionHub;
        RevBulkData bulkData;
        ExpansionHubMotor leftf, leftr, rightf, rightr;
        //RangeAutoDriver rangeAutoDriver = new RangeAutoDriver(rover, this);
        ultimateGoalRobot.getDrivetrain().resetMotorEncoders();
        ultimateGoalRobot.initialize();
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        bulkData = expansionHub.getBulkInputData();
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        //leftf = (ExpansionHubMotor) hardwareMap.dcMotor.get("leftf");
        leftr = (ExpansionHubMotor) hardwareMap.dcMotor.get("leftr");

        ultimateGoalRobot.armGripServo.close();
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
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }
        ultimateGoalRobot.getDrivetrain().setPowers(-.3, -.3);

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


    }
}
