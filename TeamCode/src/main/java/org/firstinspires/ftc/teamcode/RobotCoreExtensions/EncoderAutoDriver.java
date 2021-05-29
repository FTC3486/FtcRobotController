package org.firstinspires.ftc.teamcode.RobotCoreExtensions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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