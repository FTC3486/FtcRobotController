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
public class DriveStraight extends LinearOpMode {

    @Override
    public void runOpMode() {



        /*
        BNO055IMU imu;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

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
*/


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
        ultimateGoalRobot.getDrivetrain().setPowers(-.3, -.3);
        /*
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
        */
        encoderAutoDriver.driveToDistance(70, 1);
        ultimateGoalRobot.getDrivetrain().setPowers(0, 0);
       /*encoderAutoDriver.driveToDistance(100, 1);
        telemetry.addData("Left", ultimateGoalRobot.getDrivetrain().getLeftEncoderCount());
        telemetry.addData("Right", ultimateGoalRobot.getDrivetrain().getRightEncoderCount());
        telemetry.update();
        sleep(10000);*/
        // Delete this if it don't work ---------> Drivetrain Correction if broke - teamwork
        /*while (ultimateGoalRobot.getDrivetrain().getRightEncoderCount() < ultimateGoalRobot.getDrivetrain().getLeftEncoderCount()) {
            ultimateGoalRobot.getDrivetrain().setPowers(.5, 0);
            telemetry.addData("Left", ultimateGoalRobot.getDrivetrain().getLeftEncoderCount());
            telemetry.addData("Right", ultimateGoalRobot.getDrivetrain().getRightEncoderCount());
            telemetry.update();
        }*/

        /*
        while (opModeIsActive()) {
            ultimateGoalRobot.getDrivetrain().setPowers(1, 1);
            bulkData = expansionHub.getBulkInputData();

            telemetry.addData("Test", ultimateGoalRobot.getDrivetrain().getLeftEncoderCount());
            telemetry.addData("5v monitor", expansionHub.read5vMonitor(ExpansionHubEx.VoltageUnits.VOLTS)); //Voltage from the phone
            telemetry.addData("12v monitor", expansionHub.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS)); //Battery voltage
            //telemetry.addData("M0 velocity", bulkData.getMotorVelocity(leftf));
            telemetry.addData("M1 velocity", bulkData.getMotorVelocity(leftr));
            //telemetry.addData("M2 velocity", bulkData.getMotorVelocity(rightf));
            //telemetry.addData("M3 velocity", bulkData.getMotorVelocity(rightr));
            telemetry.addData("Module temp", expansionHub.getInternalTemperature(ExpansionHubEx.TemperatureUnits.FAHRENHEIT) + "F");
            telemetry.addData("Module over temp", expansionHub.isModuleOverTemp());
            updateTelemetry(telemetry);
        }
        ultimateGoalRobot.getDrivetrain().setPowers(0, 0);
        */

        //-----------

        // ultimateGoalRobot.getDrivetrain().setPowers(.4, .4);
        telemetry.update();
        sleep(5000);
        /*
        BNO055IMUImpl imu = new BNO055IMUImpl("a") {
            @Override
            public String getDeviceName() {
                return null;
            }

            @Override
            public Manufacturer getManufacturer() {
                return null;
            }
        };
        */

        //encoderAutoDriver.driveToDistance(-48, .4);


    }
}
