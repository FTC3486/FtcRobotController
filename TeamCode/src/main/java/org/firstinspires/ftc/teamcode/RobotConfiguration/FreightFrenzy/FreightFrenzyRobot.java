package org.firstinspires.ftc.teamcode.RobotConfiguration.FreightFrenzy;

import android.graphics.Color;
import android.text.method.Touch;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.Drivable;
import org.firstinspires.ftc.teamcode.Subsystems.VelocityMotor;
import org.firstinspires.ftc.teamcode.Subsystems.OpenCloseServo;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.ContinuousServo;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.Drivetrain;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.Initializable;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.MechanumDrivable;
import org.firstinspires.ftc.teamcode.Subsystems.ReversableMotor;
import org.firstinspires.ftc.teamcode.Subsystems.VelocityMotor;

import java.util.LinkedList;

public class FreightFrenzyRobot implements Drivable, Initializable {
    // Components
    private final Drivetrain drivetrain;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    public DcMotor extendi;
    public DcMotor arm_elevator;

    public OpenCloseServo bucketboi;

    //public TouchSensor bottom_touch;
    //public TouchSensor middle_touch;
    //public TouchSensor top_touch;

    public BNO055IMU imu;
    public LinearOpMode opmode;
    public OpMode opmode2;
    public HardwareMap hardwareMap;

    //DistanceSensor distance1;
    //DistanceSensor distance2;

    public LinkedList<TouchSensor> touch_sensors = new LinkedList<TouchSensor>();
    public int current_level = 1;
    public int desired_level = 1;
    public int manual_level = 1;

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    public FreightFrenzyRobot(HardwareMap hardwareMap_, LinearOpMode opmode_) {
        //initialize opmode and hardwareMap
        opmode = opmode_;
        hardwareMap = hardwareMap_;

        // Drivetrain
        frontLeft = hardwareMap.dcMotor.get("leftf");
        backLeft = hardwareMap.dcMotor.get("leftr");
        frontRight = hardwareMap.dcMotor.get("rightf");
        backRight = hardwareMap.dcMotor.get("rightr");

        //initialize imu
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

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

        //other motors
        final DcMotor extendi = (DcMotor) hardwareMap.dcMotor.get("extendi");
        this.extendi = extendi;
        final DcMotor arm_elevator = (DcMotor) hardwareMap.dcMotor.get("arm_elevator");
        this.arm_elevator = arm_elevator;

        //touch sensors
        touch_sensors.add( hardwareMap.get(TouchSensor.class, "bottom_touch") );
        touch_sensors.add( hardwareMap.get(TouchSensor.class, "middle_touch") );
        touch_sensors.add( hardwareMap.get(TouchSensor.class, "top_touch") );

        //Bucket Servo
        final Servo bucketboi = hardwareMap.servo.get("bucketboi");
        this.bucketboi = new OpenCloseServo(bucketboi, .01, 0.65, 0.25);
    }

    public FreightFrenzyRobot(HardwareMap hardwareMap_, OpMode opmode_) {
        //initialize opmode and hardwareMap
        opmode2 = opmode_;
        hardwareMap = hardwareMap_;

        // Drivetrain
        frontLeft = hardwareMap.dcMotor.get("leftf");
        backLeft = hardwareMap.dcMotor.get("leftr");
        frontRight = hardwareMap.dcMotor.get("rightf");
        backRight = hardwareMap.dcMotor.get("rightr");

        //initialize imu
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

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

        //other motors
        final DcMotor extendi = (DcMotor) hardwareMap.dcMotor.get("extendi");
        this.extendi = extendi;
        final DcMotor arm_elevator = (DcMotor) hardwareMap.dcMotor.get("arm_elevator");
        this.arm_elevator = arm_elevator;

        //touch sensors
        touch_sensors.add( hardwareMap.get(TouchSensor.class, "bottom_touch") );
        touch_sensors.add( hardwareMap.get(TouchSensor.class, "middle_touch") );
        touch_sensors.add( hardwareMap.get(TouchSensor.class, "top_touch") );

        //Bucket Servo
        final Servo bucketboi = hardwareMap.servo.get("bucketboi");
        this.bucketboi = new OpenCloseServo(bucketboi, 0.1, 0.8, 0.2);
    }


    @Override
    public void initialize() {
        bucketboi.initialize();
    }

    @Override
    public Drivetrain getDrivetrain() {
        return this.drivetrain;
    }

    /*
    public double distance1(){
        return distance1.getDistance(DistanceUnit.INCH);
    }

    public double distance2(){
        return distance2.getDistance(DistanceUnit.INCH);
    }*/
};


