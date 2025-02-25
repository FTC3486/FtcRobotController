package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotConfiguration.FreightFrenzy.FreightFrenzyRobot;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.EncoderAutoDriver;
//import org.openftc.revextensions2.ExpansionHubEx;
//import org.openftc.revextensions2.ExpansionHubMotor;
//import org.openftc.revextensions2.RevBulkData;

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
public class FFAutoBlueParkFancy extends LinearOpMode {

    BNO055IMU               imu;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    boolean top_wobble_goal = false;
    boolean mid_wobble_goal = false;
    boolean bottom_wobble_goal = false;

    @Override
    public void runOpMode() {


        final FreightFrenzyRobot ffRobot = new FreightFrenzyRobot(this.hardwareMap);
        final EncoderAutoDriver encoderAutoDriver = new EncoderAutoDriver(ffRobot, this);
        //ExpansionHubEx expansionHub;
        //RevBulkData bulkData;
        //ExpansionHubMotor leftf, leftr, rightf, rightr;
        //RangeAutoDriver rangeAutoDriver = new RangeAutoDriver(rover, this);
        ffRobot.getDrivetrain().resetMotorEncoders();
        ffRobot.initialize();
        //expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 3");
        //bulkData = expansionHub.getBulkInputData();
        //expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 3");
        //leftf = (ExpansionHubMotor) hardwareMap.dcMotor.get("leftf");
        //leftr = (ExpansionHubMotor) hardwareMap.dcMotor.get("leftr");

        waitForStart();

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

        //DO MOVEMENT

        //move forward into parking thing
        //encoderAutoDriver.driveStraight(15, 0.4, hardwareMap);
//        ffRobot.flySwatter.arm.setPower(1);
        sleep(2000);
//        ffRobot.flySwatter.arm.setPower(0);

        encoderAutoDriver.driveToDistance(-15, 0.4);
        telemetry.update();
        sleep(5000);


    }
}
