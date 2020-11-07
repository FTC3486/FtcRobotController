package org.firstinspires.ftc.teamcode.RobotCoreExtensions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class MechanumAutoDriver extends AutoDriver {
    private final Drivetrain drivetrain;
    private final DcMotor frontLeft;
    private final DcMotor backLeft;
    private final DcMotor frontRight;
    private final DcMotor backRight;

    public MechanumAutoDriver(MechanumDrivable hw, LinearOpMode opMode) {
        super(hw, opMode);
        drivetrain = hw.getDrivetrain();
        frontLeft = hw.getFrontLeftMotor();
        frontRight = hw.getFrontRightMotor();
        backLeft = hw.getBackLeftMotor();
        backRight = hw.getBackRightMotor();
    }

    private void resetMotorEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveForward(double distance, double inputPower) {
        resetMotorEncoders();
        frontLeft.setPower(inputPower);
        frontRight.setPower(inputPower);
        backLeft.setPower(inputPower);
        backRight.setPower(inputPower);

        while(backLeft.getCurrentPosition() <= distance && opMode.opModeIsActive()){
            opMode.telemetry.addData("FrontLeftMotorEncoder", frontLeft.getCurrentPosition());
            opMode.telemetry.addData("BackLeftMotorEncoder", backLeft.getCurrentPosition());
            opMode.telemetry.addData("FrontRightMotorEncoder", frontRight.getCurrentPosition());
            opMode.telemetry.addData("BackRightEncoder", backRight.getCurrentPosition());
            opMode.telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void driveBackwards(double distance, double inputPower) {
        resetMotorEncoders();
        frontLeft.setPower(-inputPower);
        frontRight.setPower(-inputPower);
        backLeft.setPower(-inputPower);
        backRight.setPower(-inputPower);

        while(backLeft.getCurrentPosition() >= -distance && opMode.opModeIsActive()){
            opMode.telemetry.addData("FrontLeftMotorEncoder", frontLeft.getCurrentPosition());
            opMode.telemetry.addData("BackLeftMotorEncoder", backLeft.getCurrentPosition());
            opMode.telemetry.addData("FrontRightMotorEncoder", frontRight.getCurrentPosition());
            opMode.telemetry.addData("BackRightEncoder", backRight.getCurrentPosition());
            opMode.telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void driveLeft(double distance, double inputPower) {
        resetMotorEncoders();
        frontLeft.setPower(inputPower);
        frontRight.setPower(-inputPower);
        backLeft.setPower(-inputPower);
        backRight.setPower(inputPower);

        while(backLeft.getCurrentPosition() >= -distance && opMode.opModeIsActive()){ }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void driveRight(double distance, double inputPower) {
        resetMotorEncoders();
        frontLeft.setPower(-inputPower);
        frontRight.setPower(inputPower);
        backLeft.setPower(inputPower);
        backRight.setPower(-inputPower);

        while(backLeft.getCurrentPosition() <= distance && opMode.opModeIsActive()){ }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void rotateClockwise(double distance, double inputPower) {
        resetMotorEncoders();
        frontLeft.setPower(inputPower);
        frontRight.setPower(-inputPower);
        backLeft.setPower(inputPower);
        backRight.setPower(-inputPower);

        while(backLeft.getCurrentPosition() <= distance && opMode.opModeIsActive()){ }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void rotateCounterclockwise(double distance, double inputPower) {
        resetMotorEncoders();
        frontLeft.setPower(-inputPower);
        frontRight.setPower(inputPower);
        backLeft.setPower(-inputPower);
        backRight.setPower(inputPower);

        while(backLeft.getCurrentPosition() >= -distance && opMode.opModeIsActive()){ }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}

