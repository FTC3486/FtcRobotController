package org.firstinspires.ftc.teamcode.RobotConfiguration.Mechanum;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotCoreExtensions.Drivetrain;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.Initializable;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.MechanumDrivable;

public class MechanumRobot implements MechanumDrivable, Initializable {
    // Components
    private final Drivetrain drivetrain;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    public MechanumRobot(HardwareMap hardwareMap) {
        // Drivetrain
        frontLeft = hardwareMap.dcMotor.get("leftf");
        backLeft = hardwareMap.dcMotor.get("leftr");
        frontRight = hardwareMap.dcMotor.get("rightf");
        backRight = hardwareMap.dcMotor.get("rightr");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        this.drivetrain = new Drivetrain.Builder()
                .addLeftMotorWithEncoder(backLeft)
                .addLeftMotorWithEncoder(frontLeft)
                .addRightMotorWithEncoder(frontRight)
                .addRightMotorWithEncoder(backRight)
                .setGearRatio(1.2)
                .build();
    }

    @Override
    public void initialize() {}

    @Override
    public Drivetrain getDrivetrain() {
        return this.drivetrain;
    };

    @Override
    public DcMotor getFrontLeftMotor() { return frontLeft; };

    @Override
    public DcMotor getFrontRightMotor() { return frontRight; };

    @Override
    public DcMotor getBackLeftMotor() { return backLeft; };

    @Override
    public DcMotor getBackRightMotor() { return backRight; };
};


