package org.firstinspires.ftc.teamcode.OpModes.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotConfiguration.UltimateGoal.UltimateGoalRobot;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.GamepadWrapper;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.TeleopDriver;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

//@Disabled
@TeleOp (name = "Ultimate Goal Teleop", group = "Teleop2020-21")
public class UltimateGoalTeleop extends OpMode {
    private UltimateGoalRobot ultimateGoalRobot;
    //Create joysticks
    private GamepadWrapper joy1 = new GamepadWrapper();
    private GamepadWrapper joy2 = new GamepadWrapper();
    //Create robot driver
    private TeleopDriver teleopDriver;
    //Creates elements for extra data
    ExpansionHubEx expansionHub;
    RevBulkData bulkData;
    ExpansionHubMotor leftf, leftr, rightf, rightr;
    //Creates webcam
    //Creates string for telemetry about shooter state
    String shooterState = "";
    //Variables to adjust motor velocities for deliveries
    final int powershotPower = 2225;
    //final int powershotPower = 2125;
    //final int powershotPower = 2400;
    final int towerPower = 2375;
    //final int towerPower = 2550;
    //final int towerPower = 2450;, powershotPower = 2300;

    // VuforiaDriver vuforiaDriver;

    @Override
    public void init() {
        ultimateGoalRobot = new UltimateGoalRobot(this.hardwareMap);
        ultimateGoalRobot.initialize();
        teleopDriver = new TeleopDriver(ultimateGoalRobot);
        teleopDriver.setMinSpeed(0.2);
        /*
        vuforiaDriver = new VuforiaDriver(new VuforiaCurrentGame(), getTrackableNames(), "AUWvyFD/////AAABmaAT+Xe4Iks6qo/7ra3yndh8HtUQLcEHWOxW1ZN74TVmgwOIg6aPYTrI0zh6peSAzci+oaRE3SteoHRMP3gLxfzjW98ja2voVSq1M1wLw3jFhVyd6Yv/RzYQ+HWQi+rpDkyLUv7GKOj+vBmGwxW8oyrazADHJ0a0VNoNcX1vPZhN8bwN3xRUzlG0g35012sY2CehU613sQw7GNOqmCVAEfVHBrVN6EdcYWIj5i1YpsXWb899AigohjtDu83oO1MeZQCWTinNHHMr6KUCX+Dn1wGvbJBKJrWA5liHUBaUOUScEylDa6Zp5ulY6fWr2YUszVRiTwR8WeqoATNTcbarq6psjT/sbmeX42nE/MKkRA9F");
        vuforiaDriver.initialize(VuforiaLocalizer.CameraDirection.BACK, new Position(DistanceUnit.INCH, 4, 8, 0, 0), new Orientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, 90, -90, 0, 0));
        */
        ultimateGoalRobot.initialize();
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        //leftf = (ExpansionHubMotor) hardwareMap.dcMotor.get("leftf");
        leftr = (ExpansionHubMotor) hardwareMap.dcMotor.get("leftr");
        //rightf = (ExpansionHubMotor) hardwareMap.dcMotor.get("rightf");
        rightr = (ExpansionHubMotor) hardwareMap.dcMotor.get("rightr");
    }

    /**
     * Get all trackables EXCEPT for the "STONE_TARGET" trackable, which doesn't have a single known position
     *
     * @return
     */
    /*
    private List<String> getTrackableNames() {
        return Arrays.asList(VuforiaCurrentGame.TRACKABLE_NAMES);
    }
    */
    @Override
    public void loop() {
        //Gamepad 1 is the driver controller, gamepad 2 is the gunner controller
        joy1.update(gamepad1);
        joy2.update(gamepad2);

        bulkData = expansionHub.getBulkInputData();

        //telemetry.addData("Test", ultimateGoalRobot.getDrivetrain().getLeftEncoderCount());
        // telemetry.addData("5v monitor", expansionHub.read5vMonitor(ExpansionHubEx.VoltageUnits.VOLTS)); //Voltage from the phone
        // telemetry.addData("12v monitor", expansionHub.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS)); //Battery voltage
        //telemetry.addData("M3 velocity", bulkData.getMotorVelocity(rightr));

       /* telemetry.addData("Module temp", expansionHub.getInternalTemperature(ExpansionHubEx.TemperatureUnits.FAHRENHEIT) + "F");
        telemetry.addData("Module over temp", expansionHub.isModuleOverTemp());
        telemetry.addData("Shooter", shooterState);*/
        telemetry.addData("Left Rear", leftr.getVelocity());
        telemetry.addData("Right Rear", rightr.getVelocity());
        telemetry.addData("Delivery 1", ultimateGoalRobot.delivery1.getVelocity());
        telemetry.addData("Delivery 2", ultimateGoalRobot.delivery2.getVelocity());
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
        /*
        try {
            telemetry.addData("Stone Position", vuforiaDriver.getTrackableLocation(VuforiaCurrentGame.TRACKABLE_NAMES[3]));
            telemetry.addData("Robot Position", vuforiaDriver.getRobotLocation());
            telemetry.update();
        } catch (VuforiaDriver.PositionCalculationException e) {
            telemetry.addData("Failed to get Robot position", e.getMessage());
            telemetry.update();
        }
         */

        //PICKUP Functions ------------------------------------------------------------------------

        if (gamepad2.dpad_left) {
            ultimateGoalRobot.flapper.run(.99);
            ultimateGoalRobot.belt.run(.99);
        } else if (gamepad2.right_bumper) {
            ultimateGoalRobot.flapper.reverse(-.99);
           ultimateGoalRobot.belt.reverse(-.99);
        } else {
            ultimateGoalRobot.flapper.stop();
            ultimateGoalRobot.belt.stop();
        }

        //Delivery Functions -----------------------------------------------------------------------
        if(gamepad2.dpad_left){
            ultimateGoalRobot.ringServo.setPower(0.99);
        }
        else if (gamepad2.left_bumper) {
            ultimateGoalRobot.ringServo.setPower(-0.99);
            ultimateGoalRobot.belt.reverse(-0.99);
        } else {
            ultimateGoalRobot.ringServo.setPower(0);
        }
        if (joy1.toggle.a) {
            //ultimateGoalRobot.delivery1.setPIDF(1.424, .1424, 0, 14.24);
            //ultimateGoalRobot.delivery2.setPIDF(1.424, .1424, 0, 14.24);
            ultimateGoalRobot.delivery1.setVelocity(-powershotPower);
            ultimateGoalRobot.delivery2.setVelocity(powershotPower);
            shooterState = "Powershot power";
        } else if (joy2.toggle.a) {
            //ultimateGoalRobot.delivery1.setPIDF(1.337, .1337, 0, 13.37);
            //ultimateGoalRobot.delivery2.setPIDF(1.337, .1337, 0, 13.37);
            ultimateGoalRobot.delivery1.setVelocity(-towerPower);
            ultimateGoalRobot.delivery2.setVelocity(towerPower);
            shooterState = "Tower Goal power";
        } else {
            ultimateGoalRobot.delivery1.stop();
            ultimateGoalRobot.delivery2.stop();
            shooterState = "No power";
        }

        if (joy2.toggle.x) {
        ultimateGoalRobot.armAngleServo.open();
        }   else {
            ultimateGoalRobot.armAngleServo.close();
    }
        if (joy2.toggle.b) {
            ultimateGoalRobot.armGripServo.open();
        } else {
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
        // Mary's Marvelous Magnificent Modification
        /*
        //Test Thingie to make belt run in reverse
        if(gamepad2.dpad_right){
            ultimateGoalRobot.belt.run(0.99);
        } else {
            ultimateGoalRobot.belt.stop();
        }
        */