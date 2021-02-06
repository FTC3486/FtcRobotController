package org.firstinspires.ftc.teamcode.RobotConfiguration.Test;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotCoreExtensions.Drivable;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.Drivetrain;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.Initializable;
import org.firstinspires.ftc.teamcode.Subsystems.Latch;


public class Test implements Drivable, Initializable {
    // Components
    private final Drivetrain drivetrain;
    public Latch latch;

    //Manipulator Hardware**********************
    // 2nd joint 180
    //Pickup motors for collecting Stone
    //public final ReversableMotor motor1;
   // public final ReversableMotor motor2;

    // Sensors
    //private RangeSensor leftRangeSensor;
    //private RangeSensor rightRangeSensor;
    //public final ColorSensor colorSensor;

    //DRIVETRAIN Hardware**********************


    public Test(HardwareMap hardwareMap) {
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

        //Color sensor
        //this.colorSensor = hardwareMap.colorSensor.get("colorSensor");*/
    }

    @Override
    public void initialize() {
    }

    @Override
    public Drivetrain getDrivetrain() {
        return this.drivetrain;
    };
};

