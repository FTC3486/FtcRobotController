package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotConfiguration.Mechanum.MechanumRobot;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.MechanumAutoDriver;

@Autonomous (group = "Blue" )
public class MechanumTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        final MechanumRobot mechanumRobot = new MechanumRobot(this.hardwareMap);
        final MechanumAutoDriver mechanumAutoDriver = new MechanumAutoDriver(mechanumRobot, this);

        //RangeAutoDriver rangeAutoDriver = new RangeAutoDriver(rover, this);
        mechanumRobot.initialize();

        //Autonomous for Squatty the Robot Blue Side - Park Close
        waitForStart();

        // One block is 1375 encoder counts
        mechanumAutoDriver.driveForward(1375, 1);
        sleep(5000);

        mechanumAutoDriver.driveForward(1375, 0.5);
        sleep(5000);

        mechanumAutoDriver.driveForward(1375, 0.3);
        sleep(5000);

        mechanumAutoDriver.driveBackwards(1375, 1);
        sleep(5000);

        mechanumAutoDriver.driveBackwards(1375, 0.5);
        sleep(5000);

        mechanumAutoDriver.driveBackwards(1375, 1);
        sleep(5000);
    }
}