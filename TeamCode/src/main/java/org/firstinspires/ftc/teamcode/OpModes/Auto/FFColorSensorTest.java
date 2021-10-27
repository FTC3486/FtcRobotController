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


@Autonomous (group = "Blue" )
public class FFColorSensorTest extends LinearOpMode {

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

        while (opModeIsActive()) {
            telemetry.addData("Left Color Red: ", ultimateGoalRobot.colorSensorLeft.red());
            telemetry.addData("Left Color Blue: ", ultimateGoalRobot.colorSensorLeft.blue());
            telemetry.addData("Left Color Green: ", ultimateGoalRobot.colorSensorLeft.green());
            telemetry.addData("Right Color Red: ", ultimateGoalRobot.colorSensorRight.red());
            telemetry.addData("Right Color Blue: ", ultimateGoalRobot.colorSensorRight.blue());
            telemetry.addData("Right Color Green: ", ultimateGoalRobot.colorSensorRight.green());
            updateTelemetry(telemetry);
        }

        telemetry.addData("Finished: ", true);
        telemetry.update();
        sleep(5000);
    }
}
