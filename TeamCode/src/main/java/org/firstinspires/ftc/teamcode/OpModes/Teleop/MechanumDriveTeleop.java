package org.firstinspires.ftc.teamcode.OpModes.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotCoreExtensions.GamepadWrapper;

@TeleOp(name = "Mechanum Drive Teleop", group = "Teleop")
public class MechanumDriveTeleop extends OpMode {
    private static final float CLOCKWISE_TURNING_SENSITIVITY = 0.75f;

    private GamepadWrapper joy1;
    private GamepadWrapper joy2;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private final double lowSpeedMultiplier = 0.25;
    private final double halfSpeedMultiplier = 0.5;
    private final double fullSpeedMultiplier = 1;

    @Override
    public void init() {
        joy1 = new GamepadWrapper();
        joy2 = new GamepadWrapper();

        frontLeft = hardwareMap.get(DcMotor.class, "leftf");
        frontRight = hardwareMap.get(DcMotor.class, "rightf");
        backLeft = hardwareMap.get(DcMotor.class, "leftr");
        backRight = hardwareMap.get(DcMotor.class, "rightr");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        //Gamepad 1 is the driver controller, gamepad 2 is the gunner controller
        joy1.update(gamepad1);
                joy2.update(gamepad2);

        // Create direction vector
        final float forward = -gamepad1.left_stick_y;
        final float right = gamepad1.left_stick_x;
        float clockwise = gamepad1.right_stick_x;

        // Now add a tuning constant K for the “rotate” axis sensitivity.
        // Start with K=0, and increase it very slowly (do not exceed K=1)
        // to find the right value after you’ve got fwd/rev and strafe working:
        clockwise = CLOCKWISE_TURNING_SENSITIVITY * clockwise;

        // Now apply the inverse kinematic tranformation
        // to convert your vehicle motion command
        // to 4 wheel speed commands:
        float frontLeftSpeed = forward + clockwise - right;
        float frontRightSpeed = forward - clockwise + right;
        float backLeftSpeed = forward + clockwise + right;
        float backRightSpeed = forward - clockwise - right;

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

        // Send power to wheels

        if (gamepad1.left_trigger > 0) {
            frontLeft.setPower(frontLeftSpeed * fullSpeedMultiplier);
            frontRight.setPower(frontRightSpeed * fullSpeedMultiplier);
            backLeft.setPower(backLeftSpeed * fullSpeedMultiplier);
            backRight.setPower(backRightSpeed * fullSpeedMultiplier);
        } else if (gamepad1.right_trigger > 0) {
            frontLeft.setPower(frontLeftSpeed * lowSpeedMultiplier);
            frontRight.setPower(frontRightSpeed * lowSpeedMultiplier);
            backLeft.setPower(backLeftSpeed * lowSpeedMultiplier);
            backRight.setPower(backRightSpeed * lowSpeedMultiplier);
        } else {
            frontLeft.setPower(frontLeftSpeed * halfSpeedMultiplier);
            frontRight.setPower(frontRightSpeed * halfSpeedMultiplier);
            backLeft.setPower(backLeftSpeed * halfSpeedMultiplier);
            backRight.setPower(backRightSpeed * halfSpeedMultiplier);

        }
    }
}
