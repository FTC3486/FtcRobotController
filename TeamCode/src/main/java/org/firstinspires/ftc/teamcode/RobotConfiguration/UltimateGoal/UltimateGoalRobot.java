package org.firstinspires.ftc.teamcode.RobotConfiguration.UltimateGoal;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotCoreExtensions.Drivable;
import org.firstinspires.ftc.teamcode.Subsystems.VelocityMotor;
import org.firstinspires.ftc.teamcode.Subsystems.OpenCloseServo;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.ContinuousServo;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.Drivetrain;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.Initializable;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.MechanumDrivable;
import org.firstinspires.ftc.teamcode.Subsystems.ReversableMotor;
import org.firstinspires.ftc.teamcode.Subsystems.VelocityMotor;

public class UltimateGoalRobot implements Drivable, Initializable {
    // Components
    private final Drivetrain drivetrain;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    public ReversableMotor flapper;
    public ReversableMotor belt;
    public VelocityMotor delivery1;
    public VelocityMotor delivery2;

    public OpenCloseServo armAngleServo;
    public OpenCloseServo armGripServo;
    public ContinuousServo ringServo;

    public ColorSensor colorSensorLeft;
    public ColorSensor colorSensorRight;

    public UltimateGoalRobot(HardwareMap hardwareMap) {
        // Drivetrain
        frontLeft  = hardwareMap.dcMotor.get("leftf");
        backLeft = hardwareMap.dcMotor.get("leftr");
        frontRight = hardwareMap.dcMotor.get("rightf");
        backRight = hardwareMap.dcMotor.get("rightr");


        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        this.drivetrain = new Drivetrain.Builder()
                .addLeftMotorWithEncoder(backRight)
                .addLeftMotor(frontRight)
                .addRightMotor(frontLeft)
                .addRightMotorWithEncoder(backLeft)
                .setGearRatio(1.2)
                .build();
        final DcMotorEx delivery1 = (DcMotorEx) hardwareMap.dcMotor.get("delivery1");
        this.delivery1 = new VelocityMotor(delivery1, 1);
        final DcMotorEx delivery2 = (DcMotorEx) hardwareMap.dcMotor.get("delivery2");
        this.delivery2 = new VelocityMotor(delivery2, 1);

        final Servo armAngleServo = hardwareMap.servo.get("armAngle");
        this.armAngleServo = new OpenCloseServo(armAngleServo, 0.75, 0.25, 0.703);
        //Capstone Servo
        final Servo armGripServo = hardwareMap.servo.get("armGrip");
        this.armGripServo = new OpenCloseServo(armGripServo, .01, 0.65, 0.25);

        final Servo ringServo = hardwareMap.servo.get("ringServo");
        this.ringServo = new ContinuousServo(ringServo);

        final DcMotor flapper = hardwareMap.dcMotor.get("flapper");
        this.flapper = new ReversableMotor(flapper, 1);

        final DcMotor belt = hardwareMap.dcMotor.get("belt");
        this.belt = new ReversableMotor(belt, 1);

        colorSensorLeft = hardwareMap.get(ColorSensor.class, "colorSensorLeft");

        colorSensorRight = hardwareMap.get(ColorSensor.class, "colorSensorRight");
    }


    @Override
    public void initialize() {
        armAngleServo.initialize();
        armGripServo.initialize();
    }

    @Override
    public Drivetrain getDrivetrain() {
        return this.drivetrain;
    };
/*
    @Override
    public DcMotor getFrontLeftMotor() { return frontLeft; };

    @Override
    public DcMotor getFrontRightMotor() { return frontRight; };

    @Override
    public DcMotor getBackLeftMotor() { return backLeft; };

    @Override
    public DcMotor getBackRightMotor() { return backRight; };*/
};


