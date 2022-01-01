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
import org.firstinspires.ftc.teamcode.Subsystems.DiscreteServo;
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

    public DcMotor duckSpinner;



    public BNO055IMU imu;
    public LinearOpMode opmode;
    public OpMode opmode2;
    public HardwareMap hardwareMap;

    //DistanceSensor distance_left;
    //DistanceSensor distance_middle;
    //DistanceSensor distance_right;

    /*
    public LinkedList<TouchSensor> touch_sensors = new LinkedList<TouchSensor>();

    public int current_level = 2;
    public int desired_level = 2;
    public int manual_level = 1;
        */
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

        DcMotor duckSpinner = hardwareMap.dcMotor.get("duckSpinner");
        this.duckSpinner = duckSpinner;

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
                .addLeftMotorWithEncoder(frontRight)
                .addLeftMotor(backRight)
                .addRightMotor(backLeft)
                .addRightMotorWithEncoder(frontLeft)
                .setGearRatio(1.2)
                .build();

        //other motors
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

        DcMotor duckSpinner = hardwareMap.dcMotor.get("duckSpinner");
        this.duckSpinner = duckSpinner;

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


    }

    public enum MarkerArmServoPosition implements DiscreteServo.DiscreteServoPosition {

        Initialized(.5),
        Down(0.555),
        Middle(0.68),
        Up(.7);

        private final double position;

        MarkerArmServoPosition(double position) {
            this.position = position;
        };

        @Override
        public double getPosition() {
            return position;
        };    }


    @Override
    public void initialize() {

    }

    @Override
    public Drivetrain getDrivetrain() {
        return this.drivetrain;
    }

    /*
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
        if (touch_sensors.get(2).isPressed()) {
            arm_elevator.setPower(0);
            return;
        }

        arm_elevator.setPower(-1.0);

        //wait until middle sensor is pressed
        while(!touch_sensors.get(2).isPressed()){}
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
    */

};


