package org.firstinspires.ftc.teamcode.RobotCoreExtensions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class MechanumDriver {

    private final DcMotor frontLeftMotor;
    private final DcMotor frontRightMotor;
    private final DcMotor backLeftMotor;
    private final DcMotor backRightMotor;
    private float twistSensitivity;

    public MechanumDriver(
            final DcMotor frontLeftMotor,
            final DcMotor frontRightMotor,
            final DcMotor backLeftMotor,
            final DcMotor backRightMotor,
            final float twistSensitivity
    ) {
        this.frontLeftMotor = frontLeftMotor;
        this.frontRightMotor = frontRightMotor;
        this.backLeftMotor = backLeftMotor;
        this.backRightMotor = backRightMotor;
        this.twistSensitivity = twistSensitivity;
    }

    public void drive(Gamepad gamepad) {
        drive(gamepad, 1);
    }

    public void drive(Gamepad gamepad, float speedLimit) {
        // Create direction vector
        float forward = -gamepad.left_stick_y;
        float right = gamepad.left_stick_x;
        float clockwise = gamepad.right_stick_x;

        // Now add a tuning constant K for the “rotate” axis sensitivity
        clockwise = twistSensitivity * clockwise;

        // Now apply the inverse kinematic tranformation
        // to convert your vehicle motion command
        // to 4 wheel speed commands:
        float frontLeftSpeed = forward + clockwise + right;
        float frontRightSpeed = forward - clockwise - right;
        float backLeftSpeed = forward + clockwise - right;
        float backRightSpeed = forward - clockwise + right;

        // Finally, normalize the wheel speed commands
        // so that no wheel speed command exceeds magnitude of 1:
        float max = Math.abs(frontLeftSpeed);
        if (Math.abs(frontRightSpeed) > max) max = Math.abs(frontRightSpeed);
        if (Math.abs(backLeftSpeed) > max) max = Math.abs(backLeftSpeed);
        if (Math.abs(backRightSpeed) > max) max = Math.abs(backRightSpeed);
        if (max > 1) {
            frontLeftSpeed /= max;
            frontRightSpeed /= max;
            backLeftSpeed /= max;
            backRightSpeed /= max;
        }

        // Apply the speed limit
        frontLeftSpeed *= speedLimit;
        frontRightSpeed *= speedLimit;
        backLeftSpeed *= speedLimit;
        backRightSpeed *= speedLimit;

        // Send power to wheels
        frontLeftMotor.setPower(frontLeftSpeed);
        frontRightMotor.setPower(frontRightSpeed);
        backLeftMotor.setPower(backLeftSpeed);
        backRightMotor.setPower(backRightSpeed);
    }
}
