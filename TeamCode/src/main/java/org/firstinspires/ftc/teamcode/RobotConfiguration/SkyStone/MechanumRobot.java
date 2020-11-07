package org.firstinspires.ftc.teamcode.RobotConfiguration.SkyStone;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotCoreExtensions.Initializable;

public class MechanumRobot implements Initializable {
    public static final float CLOCKWISE_TURNING_SENSITIVITY = 0.5f;

    public MechanumRobot(HardwareMap hardwareMap) {
        final DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "Front Left");
        final DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "Front Right");
        final DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "Back Left");
        final DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "Back Right");

        frontLeftMotor.setDirection(Direction.REVERSE);
        frontRightMotor.setDirection(Direction.REVERSE);
        backLeftMotor.setDirection(Direction.FORWARD);
        backRightMotor.setDirection(Direction.FORWARD);
    }

    @Override
    public void initialize() {

    }
}
