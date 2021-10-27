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
public class Rotate extends LinearOpMode {

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
        ultimateGoalRobot.getDrivetrain().setPowers(-.3, -.3);

        encoderAutoDriver.spinCounterclockwise(7, 1);

        telemetry.update();
        sleep(5000);


    }
}
