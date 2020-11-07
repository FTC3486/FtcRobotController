package org.firstinspires.ftc.teamcode.RobotConfiguration.Skystone;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotCoreExtensions.ContinuousServo;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.Drivable;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.Drivetrain;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.Initializable;
import org.firstinspires.ftc.teamcode.Subsystems.DiscreteServo;
import org.firstinspires.ftc.teamcode.Subsystems.Latch;
import org.firstinspires.ftc.teamcode.Subsystems.OpenCloseServo;
import org.firstinspires.ftc.teamcode.Subsystems.ReversableMotor;


public class SkystoneRobot implements Drivable, Initializable {
    // Components
    private final Drivetrain drivetrain;
    public Latch latch;

    //Manipulator Hardware**********************
    // 2nd joint 180
    public final OpenCloseServo middleManipulatorServo;
    //Manipulator servo
    public final OpenCloseServo manipulatorServo;
    //Capstone servo
    public final OpenCloseServo capstoneServo;
    //Platform servo
    public final OpenCloseServo platformServo;
    //PICKUP Hardware******************
    //Pickup Release Servo
    public final OpenCloseServo releaseServo;
    //Pickup motors for collecting Stone
    public final ReversableMotor pickup1;
    public final ReversableMotor pickup2;
    public final ReversableMotor armVertical;
    public final ReversableMotor flapper;

    // Sensors
    //private RangeSensor leftRangeSensor;
    //private RangeSensor rightRangeSensor;
    //public final ColorSensor colorSensor;

    //DRIVETRAIN Hardware**********************


    public SkystoneRobot(HardwareMap hardwareMap) {
        // Drivetrain
        final DcMotor left1 = hardwareMap.dcMotor.get("right1");
        final DcMotor left2 = hardwareMap.dcMotor.get("right2");
        final DcMotor right1 = hardwareMap.dcMotor.get("left1");
        final DcMotor right2 = hardwareMap.dcMotor.get("left2");
        left1.setDirection(DcMotor.Direction.REVERSE);
        left2.setDirection(DcMotor.Direction.REVERSE);
        right1.setDirection(DcMotor.Direction.FORWARD);
        right2.setDirection(DcMotor.Direction.FORWARD);
        this.drivetrain = new Drivetrain.Builder()
                .addLeftMotor(left2)
                .addLeftMotorWithEncoder(left1)
                .addRightMotor(right1)
                .addRightMotorWithEncoder(right2)
                .setGearRatio(1.2)
                .build();

        //Pickup****************************************************************
        //Pickup Release Servo
        final Servo releaseServo = hardwareMap.servo.get("releaseServo");
        this.releaseServo = new OpenCloseServo(releaseServo, 0.1, .99, 0.1);
        //Capstone Servo
        final Servo capstoneServo = hardwareMap.servo.get("capstoneServo");
        this.capstoneServo = new OpenCloseServo(capstoneServo, .01, .99, .05);
        //Platform Servo
        final Servo platformServo = hardwareMap.servo.get("platformServo");
        this.platformServo = new OpenCloseServo(platformServo, .47, .55, .47);
        //Pickup Motors
        final DcMotor pickup1 = hardwareMap.dcMotor.get("pickup1");
        this.pickup1 = new ReversableMotor(pickup1, 1);
        final DcMotor pickup2 = hardwareMap.dcMotor.get("pickup2");
        this.pickup2 = new ReversableMotor(pickup2, 1);
        final DcMotor flapper = hardwareMap.dcMotor.get("flapper");
        this.flapper = new ReversableMotor(flapper, 1);
        pickup1.setDirection(DcMotor.Direction.FORWARD);
        pickup2.setDirection(DcMotor.Direction.REVERSE);
        flapper.setDirection(DcMotor.Direction.FORWARD);

        //ARM*******************************************************************
        final DcMotor armVertical = hardwareMap.dcMotor.get("armVertical");
        this.armVertical = new ReversableMotor(armVertical, 1);

        //Manipulator Hardware**********************

        //2nd joint 180
        final Servo middleManipulatorServo = hardwareMap.servo.get("middleManipulatorServo");
        this.middleManipulatorServo = new OpenCloseServo(middleManipulatorServo, 0.23, .86, 0.23);

        //Manipulator servo
        final Servo manipulatorServo = hardwareMap.servo.get("manipulatorServo");
        this.manipulatorServo = new OpenCloseServo(manipulatorServo, .38, .38, .8);

        //Color sensor
        //this.colorSensor = hardwareMap.colorSensor.get("colorSensor");*/
    }

   @Override
    public void initialize() {
        middleManipulatorServo.initialize();
        manipulatorServo.initialize();
        capstoneServo.initialize();
        platformServo.initialize();
        releaseServo.initialize();
    }

    @Override
    public Drivetrain getDrivetrain() {
        return this.drivetrain;
    };
};

