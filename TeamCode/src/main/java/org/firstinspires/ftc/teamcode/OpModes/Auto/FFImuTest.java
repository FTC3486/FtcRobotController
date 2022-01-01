package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RobotConfiguration.FreightFrenzy.FreightFrenzyRobot;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.Drivetrain;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.EncoderAutoDriver;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

@Autonomous(group = "Blue")
public class FFImuTest extends LinearOpMode {

    private FreightFrenzyRobot ffRobot;

    //BNO055IMU               imu;

    //BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    EncoderAutoDriver encoderAutoDriver;

    @Override
    public void runOpMode() {
        ffRobot = new FreightFrenzyRobot(this.hardwareMap, this);
        encoderAutoDriver = new EncoderAutoDriver(ffRobot, this);
        telemetry.addData("Test", ffRobot.getDrivetrain().getLeftEncoderCount());
        ffRobot.getDrivetrain().resetMotorEncoders();
        ffRobot.initialize();
        //ffRobot.bucketboi.close();

        //ExpansionHubEx expansionHub;
        //RevBulkData bulkData;
        //ExpansionHubMotor leftf, leftr, rightf, rightr;
        ffRobot.getDrivetrain().resetMotorEncoders();
        ffRobot.initialize();
        //expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");

        //parameters.mode                = BNO055IMU.SensorMode.IMU;
        //parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        //parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.loggingEnabled      = false;


        //imu = hardwareMap.get(BNO055IMU.class, "imu");
        encoderAutoDriver.setImu(ffRobot.imu);
        //imu.initialize(parameters);

        while (!isStopRequested() && !ffRobot.imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", ffRobot.imu.getCalibrationStatus().toString());
        telemetry.update();
        waitForStart();
       // ffRobot.bucketboi.close();
        sleep(1000);


        //encoderAutoDriver.rotateCounterClockwise(90, hardwareMap);
        //sleep(500);

        encoderAutoDriver.driveStraight(20, 0.5, hardwareMap);
        sleep(500);

        /*encoderAutoDriver.rotateClockwise(45, hardwareMap);
        sleep(500);
        encoderAutoDriver.driveStraight(20, 0.5, hardwareMap);
        sleep(500);
        encoderAutoDriver.rotateCounterClockwise(90, hardwareMap);
        sleep(500);
        encoderAutoDriver.driveStraight(20, 0.5, hardwareMap);
        sleep(500);
        encoderAutoDriver.rotateCounterClockwise(90, hardwareMap);
        sleep(500);
        encoderAutoDriver.driveStraight(20, 0.5, hardwareMap);
        sleep(500);
        encoderAutoDriver.rotateCounterClockwise(135, hardwareMap);
        sleep(500);*/

        //drives to duck spinner
        /*encoderAutoDriver.driveStraight(5, 0.5, hardwareMap);
        sleep(500);
        encoderAutoDriver.rotateCounterClockwise(90, hardwareMap);
        sleep(500);
        encoderAutoDriver.driveStraight(5, -0.5, hardwareMap);
        sleep(500);

        //drives to code
        encoderAutoDriver.driveStraight(10, 0.5, hardwareMap);
        sleep(500);
        encoderAutoDriver.rotateClockwise(90, hardwareMap);
        sleep(500);
        encoderAutoDriver.driveStraight(5, 1, hardwareMap);
        sleep(500);
        encoderAutoDriver.driveStraight(-5, 0.1, hardwareMap);
        sleep(500);
        encoderAutoDriver.rotateCounterClockwise(90, hardwareMap);
        sleep(500);

        //drives to wobble goal
        encoderAutoDriver.driveStraight(10, 0.5, hardwareMap);
        sleep(500);
        encoderAutoDriver.rotateClockwise(90, hardwareMap);
        sleep(500);
        encoderAutoDriver.driveStraight(5, 0.5, hardwareMap);
        sleep(500);
        encoderAutoDriver.driveStraight(5, -0.5, hardwareMap);
        sleep(500);
        encoderAutoDriver.rotateCounterClockwise(90, hardwareMap);*/


    }
}
