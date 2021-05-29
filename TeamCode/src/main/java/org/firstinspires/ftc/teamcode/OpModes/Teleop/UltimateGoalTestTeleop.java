package org.firstinspires.ftc.teamcode.OpModes.Teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotConfiguration.UltimateGoal.UltimateGoalRobot;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.GamepadWrapper;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.InertialMeasurementUnit;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.TeleopDriver;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

@Disabled
@TeleOp (name = "Testing Teleop", group = "Teleop2020-21")
public class UltimateGoalTestTeleop extends OpMode {
    private UltimateGoalRobot ultimateGoalRobot;
    //Create joysticks
    private GamepadWrapper joy1 = new GamepadWrapper();
    private GamepadWrapper joy2 = new GamepadWrapper();
    private TeleopDriver teleopDriver;
    ExpansionHubEx expansionHub;
    RevBulkData bulkData;
    ExpansionHubMotor leftf, leftr, rightf, rightr;
    String shooterState = "";
    double maxVelocityOne = 0;
    double maxVelocityTwo = 0;
    @Override
    public void init() {
        ultimateGoalRobot = new UltimateGoalRobot(this.hardwareMap);
        ultimateGoalRobot.initialize();
        teleopDriver = new TeleopDriver(ultimateGoalRobot);
        teleopDriver.setMinSpeed(0.2);
        ultimateGoalRobot.initialize();
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        //leftf = (ExpansionHubMotor) hardwareMap.dcMotor.get("leftf");
        leftr = (ExpansionHubMotor) hardwareMap.dcMotor.get("leftr");
        //rightf = (ExpansionHubMotor) hardwareMap.dcMotor.get("rightf");
        rightr = (ExpansionHubMotor) hardwareMap.dcMotor.get("rightr");
    }

    @Override
    public void loop() {
        //Gamepad 1 is the driver controller, gamepad 2 is the gunner controller
        joy1.update(gamepad1);
        joy2.update(gamepad2);

        bulkData = expansionHub.getBulkInputData();

        // telemetry.addData("5v monitor", expansionHub.read5vMonitor(ExpansionHubEx.VoltageUnits.VOLTS)); //Voltage from the phone
        // telemetry.addData("12v monitor", expansionHub.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS)); //Battery voltage
        //telemetry.addData("M3 velocity", bulkData.getMotorVelocity(rightr));
        //telemetry.addData("Shooter", shooterState);
        if (ultimateGoalRobot.delivery1.getVelocity() > maxVelocityOne) {
            maxVelocityOne = ultimateGoalRobot.delivery1.getVelocity();
        }
        if (ultimateGoalRobot.delivery2.getVelocity() > maxVelocityTwo) {
            maxVelocityTwo = ultimateGoalRobot.delivery2.getVelocity();
        }
        telemetry.addData("Delivery 1 Current", ultimateGoalRobot.delivery1.getVelocity());
        telemetry.addData("Delivery 2 Current", ultimateGoalRobot.delivery2.getVelocity());

        telemetry.addData("Delivery 1 Max", maxVelocityOne);
        telemetry.addData("Delivery 2 Max", maxVelocityTwo);
        updateTelemetry(telemetry);

        //Drivetrain*******************************************************

        //Toggle Half Speed on the drivetrain
        if (gamepad1.left_trigger > 0) {
            // control the drive train at full speed
            teleopDriver.setMaxSpeed(1f);
        } else if (gamepad1.right_trigger>0){
            teleopDriver.setMaxSpeed(.3f);
        } else {
            // control the drive train at 1/2 speed - Normal driving
            teleopDriver.setMaxSpeed(.5f);
        }

        driveBackwardsToggle(joy1.toggle.right_bumper);

        //PICKUP Functions ------------------------------------------------------------------------

        if (gamepad1.dpad_left) {
            ultimateGoalRobot.flapper.run(.99);
            ultimateGoalRobot.belt.run(.99);
        } else if (gamepad2.right_bumper) {
            ultimateGoalRobot.flapper.reverse(-.99);
            ultimateGoalRobot.belt.reverse(-.99);
        } else {
            ultimateGoalRobot.flapper.stop();
            ultimateGoalRobot.belt.stop();
        }
        /*
        //Test Thingie to make belt run in reverse
        if(gamepad2.dpad_right){
            ultimateGoalRobot.belt.run(0.99);
        } else {
            ultimateGoalRobot.belt.stop();
        }
        */

        //Delivery Functions -----------------------------------------------------------------------
        if (gamepad2.left_bumper) {
            ultimateGoalRobot.ringServo.setPower(-0.99);
            ultimateGoalRobot.belt.reverse(-0.99);
        } else {
            ultimateGoalRobot.ringServo.setPower(0);
        }
        if (joy1.toggle.a) {
            ultimateGoalRobot.delivery1.run(1);
            ultimateGoalRobot.delivery2.run(1);
            shooterState = "Powershot power";
        } else if (joy1.toggle.y) {
            ultimateGoalRobot.delivery1.run(1);
            ultimateGoalRobot.delivery2.run(1);
            //ultimateGoalRobot.delivery1.reverse(-0.900);
            //ultimateGoalRobot.delivery2.reverse(0.900);
            shooterState = "Reduced power";
        } else {
            ultimateGoalRobot.delivery1.stop();
            ultimateGoalRobot.delivery2.stop();
            shooterState = "No power";
        }

        if (joy1.toggle.x) {
            ultimateGoalRobot.armAngleServo.open();
        }   else {
            ultimateGoalRobot.armAngleServo.close();
        }

        if (joy1.toggle.b) {
            ultimateGoalRobot.armGripServo.open();
        }   else {
            ultimateGoalRobot.armGripServo.close();
        }
    }

    private void driveBackwardsToggle(boolean toggle) {
        if (toggle) {
            teleopDriver.tankDrive(gamepad1, TeleopDriver.Direction.FORWARD);
        } else {
            teleopDriver.tankDrive(gamepad1, TeleopDriver.Direction.BACKWARD);
        }
    }
}