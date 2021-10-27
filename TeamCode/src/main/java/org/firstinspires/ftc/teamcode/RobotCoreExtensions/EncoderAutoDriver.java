package org.firstinspires.ftc.teamcode.RobotCoreExtensions;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Filename: EncoderAutoDriver.java
 * <p>
 * Description:
 * This class contains the methods that use the encoders for predefined autonomous movements.
 * <p>
 * Methods:
 * driveLeftSideToDistance - Drives the left side a specified distance using motor encoders
 * driveRightSideToDistance - Drives the right side a specified distance using motor encoders
 * driveToDistance - Drives both sides a specified distance forward or backwords using motor encoders
 * spinLeft - Drives the left and right side in opposite directions using motor encoders
 * spinRight - Drives the left and right side in opposite directions using motor encoders
 *
 * <p>
 * Example: hw.hardwareConfiguration.encoderAutoDriver.driveLeftSideToDistance(double distance)
 * Distances in inches.
 * <p>
 * Requirements:
 * - Drive motors with encoders
 * - An encoder auto driver is created in a hardware configuration and accessed
 * in an autonomous program for use.
 * <p>
 * Changelog:
 * -Edited and tested by Team 3486 on 7/8/2017.
 * -Edited file description and documentation 7/22/17
 * -Combined driveToDistanceForward with driveToDistanceBackwards into common program that moves both directions 7/26/18
 * -Impoved driveRightSideToDistance and Left to move forward or backwards 7/26/18
 *  - added && opMode.opModeIsActive() to each method 7/26/18
 * Updated documentation and tested each methods. 7/26/18
 */

public class EncoderAutoDriver extends AutoDriver {
    private final Drivetrain drivetrain;
    BNO055IMU imu;

    public EncoderAutoDriver(Drivable hw, LinearOpMode opMode) {
        super(hw, opMode);
        drivetrain = hw.getDrivetrain();
    }

    // Function - turnRight
    // Drives the left side and right side of the robot to turn to the right until the distanceInInches
    // is reached by the left motor encoders.
    //
    // Input – distanceInInches = the distance given in inches that the left motors will drive to
    //         leftPower = the power given to drive the left motors
    //         rightPower = the power given to drive the right motors
    //
    // Example: turnRight(12, 1.0, 0.5) -> drives until the left side of the robot has driven 12 inches
    // with the left motors set at 1.0 power and the right motors set at 0.5 power. This performs
    // a wide, sweeping turn.
    //
    // Example: turnRight(12, 1.0, -0.5) -> drives until the left side of the robot has driven 12 inches
    // with the left motors set at 1.0 power and the right motors set at -0.5 power. This performs a swivel turn.
    public void turnRight(double distanceInInches, double leftPower, double rightPower) {
        setupMotion("Turning right");

        if(leftPower > 0) {
            hw.getDrivetrain().setPowers(leftPower, rightPower);

            while(drivetrain.getLeftEncoderCount() < drivetrain.convertInchesToEncoderCounts(distanceInInches)
                    && opMode.opModeIsActive()) {}
        }
        else {
            hw.getDrivetrain().setPowers(leftPower, rightPower);

            while(drivetrain.getLeftEncoderCount() > drivetrain.convertInchesToEncoderCounts(distanceInInches)
                    && opMode.opModeIsActive()) {}
        }
        endMotion();
    }

    // Function - turnLeft
    // Drives the left side and right side of the robot to turn to the left until the distanceInInches
    // is reached by the right motor encoders.
    //
    // Input – distanceInInches = the distance given in inches that the right motors will drive to
    //         leftPower = the power given to drive the left motors
    //         rightPower = the power given to drive the right motors
    //
    // Example: turnLeft(12, 0.5, 1.0) -> drives until the right side of the robot has driven 12 inches
    // with the left motors set at 0.5 power and the right motors set at 1.0 power. This performs
    // a wide, sweeping turn.
    //
    // Example: turnLeft(12, -0.5, 1) -> drives until the right side of the robot has driven 12 inches
    // with the left motors set at -0.5 power and the right motors set at 1.0 power. This performs a swivel turn.
    public void turnLeft(double distance, double leftPower, double rightPower) {
        setupMotion("Turning left");

        if(rightPower > 0) {
            hw.getDrivetrain().setPowers(leftPower, rightPower);

            while(drivetrain.getRightEncoderCount() < drivetrain.convertInchesToEncoderCounts(distance)
                    && opMode.opModeIsActive()) {}
        } else {
            hw.getDrivetrain().setPowers(leftPower, rightPower);

            while(drivetrain.getRightEncoderCount() > drivetrain.convertInchesToEncoderCounts(distance)
                    && opMode.opModeIsActive()) {}
        }
        endMotion();
    }

    public void setImu(BNO055IMU imu){
        this.imu = imu;
    }

    private Orientation getOrientation()
    {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    //rotates vehicle to a specified global angle using PID controller and IMU
    private boolean rotatePID(float desiredAngle, HardwareMap hm){

        float starting_angle = getOrientation().firstAngle;
        float previous_angle = 0;
        float current_angle = 0;
        double target = desiredAngle;

        DcMotor frontLeft = hm.dcMotor.get("leftf");
        DcMotor frontRight = hm.dcMotor.get("rightf");
        DcMotor backLeft = hm.dcMotor.get("leftr");
        DcMotor backRight = hm.dcMotor.get("rightr");

        int totalTime = 0;

        double start_time = System.currentTimeMillis();
        double stop_counter = System.currentTimeMillis();

        double error = 90, P, I, D, kp = 0.01, ki = 0.00005, kd = 450, integral = 0, derivative, correction, t, lastTime = 0, dt = 0.1, lastError = 90;

        while(Math.abs(error) > 0.5 && opMode.opModeIsActive() && stop_counter < start_time + 5000 ){

            current_angle = getOrientation().firstAngle - starting_angle;

            //attempts to catch if we switch from 180 -> -180 or vice versa
            if(current_angle - previous_angle > 180)
                current_angle = current_angle - 360;
            if(current_angle - previous_angle < -180)
                current_angle = current_angle + 360;

            previous_angle = current_angle;

            t = (double)System.nanoTime()/10;
            if (lastTime != 0){
                dt = t - lastTime;
            }

            error = target - current_angle;

            integral = ki * ((error - lastError) * dt);
            derivative = kd * ((error - lastError) / dt);

            P = kp * error;
            I = ki * integral;
            D = kd * derivative;

            correction = P + I + D;

            if(correction > 0.5){
                correction = 0.5;
            } else if (correction < -0.5){
                correction = -0.5;
            }

            frontLeft.setPower(correction);
            frontRight.setPower(-correction);
            backLeft.setPower(correction);
            backRight.setPower(-correction);

            lastError = error;
            lastTime = t;
            totalTime += t;

            opMode.telemetry.addData("orientation: ", current_angle);
            opMode.telemetry.addData("totalTime: ", totalTime * 10E-9);
            opMode.telemetry.update();

            stop_counter = System.currentTimeMillis();
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        if(stop_counter > stop_counter + 5000){
            return false;
        } else {
            return true;
        }
    }

    public boolean rotateGlobal(float desiredAngle, HardwareMap hm){

        float starting_angle = getOrientation().firstAngle;
        float current_angle;

        while ((desiredAngle > 180 || desiredAngle < -180) && opMode.opModeIsActive()){
            if(desiredAngle > 180)
                desiredAngle -= 360;

            if(desiredAngle < -180)
                desiredAngle += 360;
        }

        double target = desiredAngle;

        DcMotor frontLeft = hm.dcMotor.get("leftf");
        DcMotor frontRight = hm.dcMotor.get("rightf");
        DcMotor backLeft = hm.dcMotor.get("leftr");
        DcMotor backRight = hm.dcMotor.get("rightr");

        int totalTime = 0;

        double start_time = System.currentTimeMillis();
        double stop_counter = System.currentTimeMillis();

        double error = 90, P, I, D, kp = 0.01, ki = 0.00005, kd = 450, integral = 0, derivative, correction, t, lastTime = 0, dt = 0.1, lastError = 90;

        while(Math.abs(error) > 0.5 && opMode.opModeIsActive() && stop_counter < start_time + 5000 ){

            current_angle = getOrientation().firstAngle;


            t = (double)System.nanoTime()/10;
            if (lastTime != 0){
                dt = t - lastTime;
            }

            error = target - current_angle;

            integral = ki * ((error - lastError) * dt);
            derivative = kd * ((error - lastError) / dt);

            P = kp * error;
            I = ki * integral;
            D = kd * derivative;

            correction = P + I + D;

            if(correction > 0.5){
                correction = 0.5;
            } else if (correction < -0.5){
                correction = -0.5;
            }

            //ffRobot.getDrivetrain().setPowers(correction,-correction);
            frontLeft.setPower(correction);
            frontRight.setPower(-correction);
            backLeft.setPower(correction);
            backRight.setPower(-correction);

            lastError = error;
            lastTime = t;
            totalTime += t;

            opMode.telemetry.addData("orientation: ", current_angle);
            opMode.telemetry.addData("totalTime: ", totalTime * 10E-9);
            opMode.telemetry.update();

            stop_counter = System.currentTimeMillis();
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        if(stop_counter > stop_counter + 5000){
            return false;
        } else {
            return true;
        }
    }


    //uses PID controller to rotate to a certain number of angles counterclockwise
    //note if angle is greater than 180, vehicle will take shortest route
    public boolean rotateCounterClockwise(float angle, HardwareMap hm){
        while ((angle > 180 || angle < -180) && opMode.opModeIsActive()){
            if(angle > 180)
                angle -= 360;

            if(angle < -180)
                angle += 360;
        }

        return rotatePID(angle, hm);
    }

    //uses PID controller to rotate to a certain number of angles clockwise
    //note if angle is greater than 180, vehicle will take shortest route
    public boolean rotateClockwise(float angle, HardwareMap hm){

        while ((angle > 180 || angle < -180) && opMode.opModeIsActive()){
            if(angle > 180)
                angle -= 360;

            if(angle < -180)
                angle += 360;
        }

        return rotatePID(-angle, hm);
    }

    //uses PID controller to drive straight
    //it accepts positive inches, positive or negative power (forward/backward) and a hardwaremap
    public boolean driveStraight(float inches, double power, HardwareMap hm){

        //we might have to pass in DriveTrain object instead of using hw, test if it works
        hw.getDrivetrain().resetMotorEncoders();
        float starting_angle = getOrientation().firstAngle;
        Drivetrain DT= hw.getDrivetrain();

        float current_angle;
        double target = starting_angle;

        double start_time = System.currentTimeMillis();
        double stop_counter = System.currentTimeMillis();

        //hack to allow traveling the -180/180 line
        boolean add_360 = false;
        boolean sub_360 = false;
        if (target > 170){
            add_360 = true;
        }
        if(target < -170){
            sub_360 = true;
        }
        //

        DcMotor frontLeft = hm.dcMotor.get("leftf");
        DcMotor frontRight = hm.dcMotor.get("rightf");
        DcMotor backLeft = hm.dcMotor.get("leftr");
        DcMotor backRight = hm.dcMotor.get("rightr");

        int totalTime = 0;

        double error = 90, P, I, D, kp = 0.01, ki = 0.00005, kd = 450, integral = 0, derivative, correction, t, lastTime = 0, dt = 0.1, lastError = 90;

        while(opMode.opModeIsActive() && stop_counter < start_time + 5000){

            //we are going forward
            if(DT.getLeftEncoderCount() > DT.convertInchesToEncoderCounts(inches) && power > 0)
                break;
            //we are going backward
            if(DT.getLeftEncoderCount() < -1*DT.convertInchesToEncoderCounts(inches) && power < 0)
                break;

            //hack to handle -180/180 line
            current_angle = getOrientation().firstAngle;
            if(add_360 && current_angle < 0)
                current_angle+=360;
            if(sub_360 && current_angle > 0)
                current_angle-=360;
            //

            t = (double)System.nanoTime()/10;
            if (lastTime != 0){
                dt = t - lastTime;
            }

            error = target - current_angle;

            integral = ki * ((error - lastError) * dt);
            derivative = kd * ((error - lastError) / dt);

            P = kp * error;
            I = ki * integral;
            D = kd * derivative;

            correction = P + I + D;

            if(correction > 0.5){
                correction = 0.5;
            } else if (correction < -0.5){
                correction = -0.5;
            }

            //corrections should be very small
            frontLeft.setPower(power + correction);
            frontRight.setPower(power - correction);
            backLeft.setPower(power + correction);
            backRight.setPower(power - correction);

            lastError = error;
            lastTime = t;
            totalTime += t;

            opMode.telemetry.addData("orientation: ", current_angle);
            opMode.telemetry.addData("totalTime: ", totalTime * 10E-9);
            opMode.telemetry.update();

            stop_counter = System.currentTimeMillis();
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        if(stop_counter > stop_counter + 5000){
            return false;
        } else {
            rotateGlobal(starting_angle, hm);
            return true;
        }
    }

    // Function - driveToDistance
    // Drives both sides straight forwards or backwards converting our inches input to counts while the OpMode is active
    // using the power variable for speed control.
    //
    // Input – distance = the distance given in inches that we want to drive in a straight line
    //       - leftAndRightPower = the motor power to apply to both the left and right sides of the robot
    //
    // Example driveToDistance(5, 1) = drive the robot straight for 5 inches at power = 1
    // driveToDistance(-5, 1) = drive the robot in reverse in a straight line for 5 inches at power = 1
    public void driveToDistance(double distance, double leftAndRightPower) {
        setupMotion("Driving to set distance.");

        if (distance > 0) {
            drivetrain.setPowers(leftAndRightPower, leftAndRightPower);
            while (drivetrain.getLeftEncoderCount() < drivetrain.convertInchesToEncoderCounts(distance)
                    && opMode.opModeIsActive()) {}
        } else {
            drivetrain.setPowers(-leftAndRightPower, -leftAndRightPower);
            while (drivetrain.getLeftEncoderCount() > drivetrain.convertInchesToEncoderCounts(distance)
                    && opMode.opModeIsActive()) {}
        }
        endMotion();
    }

    // Function - spinClockwise
    // Spins the robot clockwise by applying the same magnitude of motor power to either
    // side of the robot until the leftInches distance is reached by the left side of the robot.
    //
    // Input - distance = the distance given in inches that we want the left side of the robot to spin
    //       - leftPower = the power that we want to give the left side of the robot. This is also used to
    //                     set the right power.
    //
    // Examples - spinClockwise(7, 1) - Spin the robot clockwise at power = 1 until the left
    //             side of the robot has traveled 7 inches.
    public void spinClockwise(double distanceInInches, double leftPower) {
        turnRight(distanceInInches, leftPower, -leftPower);
    }

    // Function - spinCounterclockwise
    // Spins the robot counterclockwise by applying the same magnitude of motor power to either
    // side of the robot until the rightInches distance is reached by the right side of the robot.
    //
    // Input - distance = the distance given in inches that we want the left side of the robot to spin
    //       - rightPower = the power that we want to give the right side of the robot. This is also used to
    //                     set the left power.
    //
    // Examples - spinCounterclockwise(7, 1) - Spin the robot clockwise at power = 1 until the right
    //             side of the robot has traveled 7 inches.
    public void spinCounterclockwise(double distance, double rightPower) {
        turnLeft(distance, -rightPower, rightPower);
    }

    public void coast(double distance, double leftPower, double rightPower) {
        setupMotion("Turning left");

        if (rightPower > 0) {
            hw.getDrivetrain().setPowers(leftPower, rightPower);

            while(drivetrain.getRightEncoderCount() < drivetrain.convertInchesToEncoderCounts(distance)
                    && opMode.opModeIsActive()) {}
        } else {
            hw.getDrivetrain().setPowers(leftPower, rightPower);

            while(drivetrain.getRightEncoderCount() > drivetrain.convertInchesToEncoderCounts(distance)
                    && opMode.opModeIsActive()) {}
        }
    }
    public void coastStop(double distance, double leftPower, double rightPower) {
        setupMotion("Turning left");

        if (rightPower > 0) {
            hw.getDrivetrain().setPowers(leftPower, rightPower);

            while(drivetrain.getRightEncoderCount() < drivetrain.convertInchesToEncoderCounts(distance)
                    && opMode.opModeIsActive()) {}
        } else {
            hw.getDrivetrain().setPowers(leftPower, rightPower);

            while(drivetrain.getRightEncoderCount() > drivetrain.convertInchesToEncoderCounts(distance)
                    && opMode.opModeIsActive()) {}
        }
        endMotion();
    }
}