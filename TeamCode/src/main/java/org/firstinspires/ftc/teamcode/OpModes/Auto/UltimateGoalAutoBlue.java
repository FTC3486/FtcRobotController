package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotConfiguration.UltimateGoal.UltimateGoalRobot;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.EncoderAutoDriver;

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
public class UltimateGoalAutoBlue extends LinearOpMode {
    @Override
    public void runOpMode() {
        final UltimateGoalRobot ultimateGoalRobot = new UltimateGoalRobot(this.hardwareMap);
        final EncoderAutoDriver encoderAutoDriver = new EncoderAutoDriver(ultimateGoalRobot, this);
        telemetry.addData("Test", ultimateGoalRobot.getDrivetrain().getLeftEncoderCount());
        //RangeAutoDriver rangeAutoDriver = new RangeAutoDriver(rover, this);
        ultimateGoalRobot.getDrivetrain().resetMotorEncoders();
        ultimateGoalRobot.initialize();
        ultimateGoalRobot.armGripServo.close();
        waitForStart();


        encoderAutoDriver.setPower(1);
        encoderAutoDriver.driveToDistance(80, 1);
        ultimateGoalRobot.armAngleServo.open();
        ultimateGoalRobot.armGripServo.open();
        encoderAutoDriver.driveToDistance(-25, 1);
        ultimateGoalRobot.belt.reverse(-0.99);
        ultimateGoalRobot.ringServo.setPower(-0.99);
        ultimateGoalRobot.delivery1.reverse(-0.99);
        ultimateGoalRobot.delivery2.run(0.99);
        sleep(10000);

    }
}
