package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotConfiguration.FreightFrenzy.FreightFrenzyRobot;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.EncoderAutoDriver;
//import org.openftc.revextensions2.ExpansionHubEx;
//import org.openftc.revextensions2.ExpansionHubMotor;
//import org.openftc.revextensions2.RevBulkData;

/*
    Filename: UltimateGoalAutoBlue.java
    Description:
        Autonomous program for the Ultimate Goal season on the Blue team.

  * Changelog:
 *     -
 */

@Autonomous (group = "Blue" )
public class FFAutoBluePark extends LinearOpMode {

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
        /*
        int markerPos;

        int i = 0;
        int pos1 = 0;
        int pos2 = 0;
        int pos3 = 0;

        while((i < 100) && opModeIsActive())
        {
            double dis1 = ffRobot.distance1.getDistance(DistanceUnit.INCH);
            double dis2 = ffRobot.distance3.getDistance(DistanceUnit.INCH);

            if ((dis1 < 12) && (dis2  > 12)) {
                pos1++;
            } else if ((dis1 > 12) && (dis2  < 12)) {
                pos2++;
            } else {
                pos3++;
            }
            telemetry.addData("pos1: ", pos1);
            telemetry.addData("pos2: ", pos2);
            telemetry.addData("pos3: ", pos3);
            telemetry.addData("distance2: ", dis2);
            telemetry.addData("distance1: ", dis1);
            telemetry.update();
            i++;
        }

        if((pos1 > pos2) && (pos1 > pos3))
        {
            markerPos = 1;
        }
        else if((pos2 > pos1) && (pos2 > pos3))
        {
            markerPos = 2;
        }
        else if((pos3 > pos1) && (pos3 > pos2))
        {
            markerPos = 3;
        }
        else {
            markerPos = 3;
        }
        */


        //ffRobot.flySwatter.arm.setPower(1);
       // sleep(2000);
        //ffRobot.flySwatter.arm.setPower(0);

        //ffRobot.flySwatter.wrist.setPosition(0.4);

        encoderAutoDriver.turnRight(9, 0.1, 0.6);

        encoderAutoDriver.driveToDistance(6, 0.6);

        encoderAutoDriver.rotateClockwise(90, hardwareMap);

//        double armPos = ffRobot.flySwatter.arm.getCurrentPosition();
//
//        while(ffRobot.flySwatter.arm.getCurrentPosition() < 5100)
//        {
//            ffRobot.flySwatter.arm.setPower(0.9);
//        }
//
//        ffRobot.flySwatter.arm.setPower(0);

        // what we need:
        // ffRobot.flySwatter.wrist.setPosition(0.800);

       //encoderAutoDriver.rotateClockwise(180, hardwareMap);

        //encoderAutoDriver.driveToDistance(5, .6);
        /*
        ffRobot.flySwatter.flapperForward();
        sleep(1000);
        ffRobot.flySwatter.flapperStop();

        encoderAutoDriver.turnRight(10, 0.6, 0);

        encoderAutoDriver.driveToDistance(15, 0.4);
        telemetry.update();
        sleep(5000);
        */



    }
}
