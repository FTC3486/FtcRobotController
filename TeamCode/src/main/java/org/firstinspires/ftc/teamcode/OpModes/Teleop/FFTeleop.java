package org.firstinspires.ftc.teamcode.OpModes.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotConfiguration.FreightFrenzy.FreightFrenzyRobot;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.GamepadWrapper;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.TeleopDriver;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

//@Disabled
@TeleOp (name = "FF Goal Teleop", group = "Teleop2020-21")
public class FFTeleop extends OpMode {
    private FreightFrenzyRobot ffRobot;
    //Create joysticks
    private GamepadWrapper joy1 = new GamepadWrapper();
    private GamepadWrapper joy2 = new GamepadWrapper();
    //Create robot driver
    private TeleopDriver teleopDriver;
    //Creates elements for extra data
    ExpansionHubEx expansionHub;
    RevBulkData bulkData;
    ExpansionHubMotor leftf, leftr, rightf, rightr;

    //elevator bools
    boolean initialize_elevator;
    boolean go_down_init;
    boolean new_level;
    boolean manual_override;


    @Override
    public void init() {
        ffRobot = new FreightFrenzyRobot(this.hardwareMap, this);
        ffRobot.initialize();
        teleopDriver = new TeleopDriver(ffRobot);
        teleopDriver.setMinSpeed(0.2);

        ffRobot.initialize();

        //telemetry output
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 3");
        leftf = (ExpansionHubMotor) hardwareMap.dcMotor.get("leftf");
        rightf = (ExpansionHubMotor) hardwareMap.dcMotor.get("rightf");

        //automatic elevator bools
        initialize_elevator = true;
        go_down_init = true;
        new_level = false;

        //manual elevator bools
        manual_override = false;
    }

    @Override
    public void loop() {
        //INITIALIZE ELEVATOR
        //check if we are already in middle position
        if((initialize_elevator && ffRobot.touch_sensors.get(1).isPressed() || manual_override) ){
            initialize_elevator = false;
            go_down_init = false;
            ffRobot.arm_elevator.setPower(0);
        }

        //if elevator is not already in middle position, go down to bottom and go to middle
        if(initialize_elevator && go_down_init) {
            ffRobot.arm_elevator.setPower(1.0);
            go_down_init = false;
        }

        if( (initialize_elevator && ffRobot.touch_sensors.get(0).isPressed())){
            ffRobot.arm_elevator.setPower(-1.0);
        }
        //-----------------------------------------------------------------
        //Gamepad 1 is the driver controller, gamepad 2 is the gunner controller
        joy1.update(gamepad1);
        joy2.update(gamepad2);

        bulkData = expansionHub.getBulkInputData();

        telemetry.addData("Left Rear", leftf.getVelocity());
        telemetry.addData("Right Rear", rightf.getVelocity());

        //Drivetrain*******************************************************

        teleopDriver.tankDrive(gamepad1, TeleopDriver.Direction.BACKWARD);
        //Toggle Half Speed on the drivetrain
        if (gamepad1.left_trigger > 0) {
            // control the drive train at full speed
            teleopDriver.setMaxSpeed(1f);
        } else if (gamepad1.right_trigger > 0) {
            teleopDriver.setMaxSpeed(.3f);
        } else {
            // control the drive train at 1/2 speed - Normal driving
            teleopDriver.setMaxSpeed(.5f);
        }

        //PICKUP Functions ------------------------------------------------------------------------

        /*if(joy2.onPress.dpad_up){ //pickup
            ffRobot.flappy_bird.setPower(-1.0);
        }
        if(joy2.onPress.dpad_left){ //stop
            ffRobot.flappy_bird.setPower(0);
        }
        if(joy2.onPress.dpad_down){ //throw out
            ffRobot.flappy_bird.setPower(1.0);
        }*/

        //DELIVERY Functions ------------------------------------------------------------------------

        //SET ELEVATOR-----------------------------
        /* Diagram to understand terminology
        * touch sensor 2
        * manual level 1 (space between top and mid touch sensors)
        * touch sensor 1
        * manual level 0 (space between mid and bottom touch sensors)
        * touch sensor 0
        *
        * Note:
        * current_level corresponds to the touch sensors levels, not the manual levels
        * new_level is only true during automatic elevator and becomes false when we reach goal.
        * manual_override is only true if a trigger is hit. it remains true until a, x, or y are pressed.
        *
        * Note2:
        * if you wish to disable automatic elevator, simply comment out everything in this section
        * after STEP1 (except STEP4a)
        * */

        //STEP1: Listen to whichever button we press
        if(joy2.onPress.a){ //go to level 0
            if((!new_level && ffRobot.current_level != 0) || manual_override) {
                new_level = true;
                ffRobot.desired_level = 0;
            }
        } else if (joy2.onPress.x) { //go to level 1

            if((!new_level && ffRobot.current_level != 1) || manual_override) {
                new_level = true;
                ffRobot.desired_level = 1;
            }
        }else if (joy2.onPress.y) { //go to level 2
            if ((!new_level && ffRobot.current_level != 2) || manual_override) {
                new_level = true;
                ffRobot.desired_level = 2;
            }
        }else if(gamepad2.left_trigger > 0.5){ //go up
            if(!ffRobot.touch_sensors.get(2).isPressed()) {
                manual_override = true;
                new_level = false;
                ffRobot.arm_elevator.setPower(-1);
            }else{
                ffRobot.arm_elevator.setPower(0);
            }
        }else if(gamepad2.right_trigger > 0.5){ //go down
            if(!ffRobot.touch_sensors.get(0).isPressed()) {
                manual_override = true;
                new_level = false;
                ffRobot.arm_elevator.setPower(1);
            }else{
                ffRobot.arm_elevator.setPower(0);
            }
        }

        //STEP2: help determine where we are exactly at all times
        if(ffRobot.touch_sensors.get(1).isPressed()) {
            ffRobot.current_level = 1;
            if(ffRobot.arm_elevator.getPower() < 0){
                ffRobot.manual_level = 1;
            }
            if(ffRobot.arm_elevator.getPower() > 0){
                ffRobot.manual_level = 0;
            }
        } else if(ffRobot.touch_sensors.get(2).isPressed()){
            ffRobot.current_level = 2;
        } else if (ffRobot.touch_sensors.get(0).isPressed()){
            ffRobot.current_level = 0;
        }

        //STEP3a: for automatic elevator make arm move in direction we want
        //ignored for manual elevator
        if(ffRobot.current_level != ffRobot.desired_level && !manual_override){

            if(ffRobot.desired_level > ffRobot.current_level){
                ffRobot.arm_elevator.setPower(-1);
            }

            if(ffRobot.desired_level < ffRobot.current_level){
                ffRobot.arm_elevator.setPower(1);
            }
        }

        //STEP3b: for transition from manual to automatic elevator, make arm move in direction we want
        //ignored for automatic elevator
        if(manual_override && new_level){
            manual_override = false;
            if(ffRobot.desired_level > ffRobot.manual_level){ //go up
                ffRobot.arm_elevator.setPower(-1);
            }

            if(ffRobot.desired_level <= ffRobot.manual_level){ //go down
                ffRobot.arm_elevator.setPower(1);
            }
        }

        //STEP4a: stop elevator if triggers aren't pressed and manual override is active
        //ignored for automatic elevator
        if(manual_override && gamepad2.left_trigger < 0.5
                && gamepad2.right_trigger < 0.5 && ffRobot.arm_elevator.getPower() != 0){
            ffRobot.arm_elevator.setPower(0);
        }

        //STEP4b: stop the automatic elevator when it reaches its destination
        // ignored for manual elevator (except for transition between manual to automatic)
        if(new_level && ffRobot.touch_sensors.get(ffRobot.desired_level).isPressed()){
            ffRobot.arm_elevator.setPower(0);
            new_level = false;
        }

        //--------------------------------

        //extender motor
        boolean left_bumper = false;
        boolean right_bumper = false;
        if(gamepad2.left_bumper && !gamepad2.right_bumper && !left_bumper){
            ffRobot.extendi.setPower(0.5);
            left_bumper = true;
        }
        else if (gamepad2.right_bumper && !gamepad2.left_bumper && !right_bumper) {
            ffRobot.extendi.setPower(-0.5);
            right_bumper = true;
        }else{
            ffRobot.extendi.setPower(0);
            left_bumper = false;
            right_bumper = false;
        }

        //bucket servo
        if (joy2.toggle.b) {
            ffRobot.bucketboi.open();
        } else if (!joy2.toggle.b){
            ffRobot.bucketboi.close();
        }

        telemetry.update();
    }
}