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
    public DcMotor flappy_bird;

    public OpenCloseServo bucketboi;
    public ContinuousServo duck_spinner;

    public BNO055IMU imu;
    public LinearOpMode opmode;
    public OpMode opmode2;
    public HardwareMap hardwareMap;

    DistanceSensor distance_left;
    DistanceSensor distance_middle;
    DistanceSensor distance_right;

    public LinkedList<TouchSensor> touch_sensors = new LinkedList<TouchSensor>();
    public int current_level = 2;
    public int desired_level = 2;
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

        //distance sensors
        distance_left = hardwareMap.get(DistanceSensor.class, "left_distance_sensor");
        //distance_middle = hardwareMap.get(DistanceSensor.class, "middle_distance_sensor");
        distance_right = hardwareMap.get(DistanceSensor.class, "right_distance_sensor");

        //Bucket Servo
        final Servo bucketboi = hardwareMap.servo.get("bucketboi");
        this.bucketboi = new OpenCloseServo(bucketboi, 0.01, 0.5, 0.01);


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

        //final DcMotor flappy_bird = (DcMotor) hardwareMap.dcMotor.get("flappy_bird");
        //this.flappy_bird = flappy_bird;

        //touch sensors
        touch_sensors.add( hardwareMap.get(TouchSensor.class, "bottom_touch") );
        touch_sensors.add( hardwareMap.get(TouchSensor.class, "middle_touch") );
        touch_sensors.add( hardwareMap.get(TouchSensor.class, "top_touch") );

        //Bucket Servo
        final Servo bucketboi = hardwareMap.servo.get("bucketboi");
        this.bucketboi = new OpenCloseServo(bucketboi, 0.05, 0.8, 0.05);

        //final Servo duck_spinner = hardwareMap.servo.get("duck_spinner");
        //this.duck_spinner = new ContinuousServo(duck_spinner);
    }


    @Override
    public void initialize() {
        bucketboi.initialize();
    }

    @Override
    public Drivetrain getDrivetrain() {
        return this.drivetrain;
    }


    public double distance_left(){
        return distance_left.getDistance(DistanceUnit.INCH);
    }

    public double distance_middle(){
        return distance_middle.getDistance(DistanceUnit.INCH);
    }

    public double distance_right(){
        return distance_right.getDistance(DistanceUnit.INCH);
    }

    //to be used only in autonomous
    public void initialize_elevator(){
        if (touch_sensors.get(1).isPressed()) {
            arm_elevator.setPower(0);
            return;
        }

        //if elevator is not already in middle position, go down to bottom and go to middle
        arm_elevator.setPower(1.0);
        while(!touch_sensors.get(0).isPressed()){
            if(touch_sensors.get(1).isPressed()){
                return;
            }
        }
        arm_elevator.setPower(-1.0);

        //wait until middle sensor is pressed
        while(!touch_sensors.get(1).isPressed()){}
        return;

    }
    //to be used only in autonomous
    public void setArm_elevator(int desired_level_){
        desired_level = desired_level;

        if(desired_level > current_level){ //go up
            arm_elevator.setPower(-1);
        }

        if(desired_level < current_level){ //go down
            arm_elevator.setPower(1);
        }
        while(!touch_sensors.get(desired_level).isPressed()){ //stay
            arm_elevator.setPower(0);
        }
    }
};


