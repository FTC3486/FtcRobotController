package org.firstinspires.ftc.teamcode.RobotConfiguration.Skystone;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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
    //Stone angle servo
    public final DiscreteServo<AngleServoPosition> angleServo;
    // 2nd joint 180
    public final OpenCloseServo middleManipulatorServo;
    //Manipulator servo
    public final OpenCloseServo manipulatorServo;
    //Capstone servo
    public final OpenCloseServo capstoneServo;
    //Stone grabber
    public final OpenCloseServo grabberServo;
    //ARM Hardware**************************

    //Arm motor for extending/retracting
    //public final ReversableMotor armVertical;


    //PICKUP Hardware******************
    //Pickup Release Servo
    public final OpenCloseServo releaseServo;
    //Pickup motors for collecting Stone
    public final ReversableMotor pickup1;
    public final ReversableMotor pickup2;
    public final ReversableMotor armVertical;
    public final ReversableMotor armHorizontal;

    //Flapper servo for tilting
    //public final SpeedServo flapperServo;


    // Sensors
    //private RangeSensor leftRangeSensor;
    //private RangeSensor rightRangeSensor;
    //public final ColorSensor colorSensor;

    //DRIVETRAIN Hardware**********************


    public SkystoneRobot(HardwareMap hardwareMap) {
        // Drivetrain
        final DcMotor left1 = hardwareMap.dcMotor.get("left1");
        final DcMotor left2 = hardwareMap.dcMotor.get("left2");
        final DcMotor right1 = hardwareMap.dcMotor.get("right1");
        final DcMotor right2 = hardwareMap.dcMotor.get("right2");
        left1.setDirection(DcMotor.Direction.FORWARD);
        left2.setDirection(DcMotor.Direction.FORWARD);
        right1.setDirection(DcMotor.Direction.REVERSE);
        right2.setDirection(DcMotor.Direction.REVERSE);
        this.drivetrain = new Drivetrain.Builder()
                .addLeftMotor(left1)
                .addLeftMotorWithEncoder(left2)
                .addRightMotor(right1)
                .addRightMotorWithEncoder(right2)
                .build();

        //Pickup****************************************************************
        //Pickup Release Servo
        final Servo releaseServo = hardwareMap.servo.get("releaseServo");
        this.releaseServo = new OpenCloseServo(releaseServo, 0.1, .9, 0.0);

        //Capstone Servo
        final Servo capstoneServo = hardwareMap.servo.get("capstoneServo");
        this.capstoneServo = new OpenCloseServo(capstoneServo, .1, .9, .3);
        //Pickup Motors
        final DcMotor pickup1 = hardwareMap.dcMotor.get("pickup1");
        this.pickup1 = new ReversableMotor(pickup1, 1);
        final DcMotor pickup2 = hardwareMap.dcMotor.get("pickup2");
        this.pickup2 = new ReversableMotor(pickup2, 1);
        //pickupMotor1.setDirection(DcMotor.Direction.FORWARD);
        // pickupMotor2.setDirection(DcMotor.Direction.REVERSE);

        //ARM*******************************************************************
        final DcMotor armVertical = hardwareMap.dcMotor.get("armVertical");
        this.armVertical = new ReversableMotor(armVertical, 1);

        final DcMotor armHorizontal = hardwareMap.dcMotor.get("armHorizontal");
        this.armHorizontal = new ReversableMotor(armHorizontal, 1);
        // Latch
/*
        final DcMotor latchMotor = hardwareMap.dcMotor.get("latch");
        final DcMotor latchMotor2 = hardwareMap.dcMotor.get("latch2");
        final DigitalChannel latchTop = hardwareMap.digitalChannel.get("latchTop");
        final DigitalChannel latchBottom = hardwareMap.digitalChannel.get("latchBottom");
        this.latch = new Latch(latchMotor,latchMotor2, latchTop, latchBottom,-1.00, 1.00);

        //Flapper motor
        final DcMotor flapperMotor = hardwareMap.dcMotor.get("flapper");
        this.flapperMotor = new ReversableMotor(flapperMotor, 1);
        //Flapper servo
        final Servo flapperServo = hardwareMap.servo.get("flapperServo");
        this.flapperServo = new SpeedServo(flapperServo, 1.0, 0.01);*/

        //Manipulator Hardware**********************
        //Stone Grabber
        final Servo grabberServo = hardwareMap.servo.get("grabberServo");
        this.grabberServo = new OpenCloseServo(grabberServo, .1, .9, .2);

        //2nd joint 180
        final Servo middleManipulatorServo = hardwareMap.servo.get("middleManipulatorServo");
        this.middleManipulatorServo = new OpenCloseServo(middleManipulatorServo, 0.1, .9, 0.0);

        //Stone angle servo
        final Servo angleServo = hardwareMap.servo.get("angleServo");
        this.angleServo = new DiscreteServo<>(angleServo, AngleServoPosition.POSITION_1, AngleServoPosition.values());

        //Manipulator servo
        final Servo manipulatorServo = hardwareMap.servo.get("manipulatorServo");
        this.manipulatorServo = new OpenCloseServo(manipulatorServo, .1, .9, .1);
        //Color sensor
        //this.colorSensor = hardwareMap.colorSensor.get("colorSensor");*/
    }

    @Override
    public void initialize() {
        //latch.initialize();
        //flapperServo.initialize();
        //markerServo.initialize();
    }

    @Override
    public Drivetrain getDrivetrain() {
        return this.drivetrain;
    }

    /*public boolean foundYellowObject(){
        //Base number 5
        return ((colorSensor.red() >= colorSensor.blue() + 10) && (colorSensor.green() >= colorSensor.blue() && colorSensor.green() <= colorSensor.red()));
    }*/


    public enum AngleServoPosition implements DiscreteServo.DiscreteServoPosition {
        POSITION_1(0.1),
        POSITION_2(0.2),
        POSITION_3(0.2),
        POSITION_4(0.2);

        private final double position;

        AngleServoPosition(double position) {
            this.position = position;
        }

        @Override
        public double getPosition() {
            return position;
        }
    }
}

