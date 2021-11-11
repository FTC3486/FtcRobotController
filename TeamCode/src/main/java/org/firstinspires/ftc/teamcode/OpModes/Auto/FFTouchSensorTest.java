package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Autonomous
public class FFTouchSensorTest extends LinearOpMode{

    // Define variables for our touch sensor and motor
    TouchSensor touch;

    @Override
    public void runOpMode() {
        //setup
        /*ffRobot = new FreightFrenzyRobot(this.hardwareMap, this);
        encoderAutoDriver = new EncoderAutoDriver(ffRobot, this);
        telemetry.addData("Test", ffRobot.getDrivetrain().getLeftEncoderCount());
        ffRobot.getDrivetrain().resetMotorEncoders();
        ffRobot.initialize();
        ffRobot.armGripServo.close();
        encoderAutoDriver.setImu(ffRobot.imu);*/
        //waitForStart();


        // Get the touch sensor and motor from hardwareMap
        TouchSensor touch1 = hardwareMap.get(TouchSensor.class, "Touch1");
        TouchSensor touch2 = hardwareMap.get(TouchSensor.class, "Touch2");
        TouchSensor touch3 = hardwareMap.get(TouchSensor.class, "Touch3");

        // Wait for the play button to be pressed
        waitForStart();

        // Loop while the Op Mode is running
        /*while (opModeIsActive()) {
            // If the touch sensor is pressed, stop the motor
            if (touch.isPressed()) {
                motor.setPower(0);
            } else { // Otherwise, run the motor
                motor.setPower(0.3);
            }
        }*/
        while(opModeIsActive()) {
            if (touch1.isPressed()) {
                telemetry.addData("touch1: ", touch1.getValue());
            }
            if (touch2.isPressed()) {
                telemetry.addData("touch2: ", touch2.getValue());
            }
            if (touch3.isPressed()) {
                telemetry.addData("touch3: ", touch3.getValue());
            }
            telemetry.update();
        }
    }
}