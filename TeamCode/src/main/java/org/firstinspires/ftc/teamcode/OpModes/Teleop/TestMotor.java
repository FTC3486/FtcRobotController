package org.firstinspires.ftc.teamcode.OpModes.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotCoreExtensions.Drivable;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.Drivetrain;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.Initializable;
import org.firstinspires.ftc.teamcode.Subsystems.Latch;

@TeleOp(name = "Test Motor", group = "Teleop2021")
public class TestMotor extends OpMode {
    //Declare parts of the robot that will be used by this Teleop

    DcMotor left1;
    //final DcMotor left2 = hardwareMap.dcMotor.get("right2");
    //final DcMotor right1 = hardwareMap.dcMotor.get("left1");
    //final DcMotor right2 = hardwareMap.dcMotor.get("left2");

    @Override
    public void init() {
        left1 = hardwareMap.dcMotor.get("right1");
        left1.setDirection(DcMotor.Direction.REVERSE);
        //left2.setDirection(DcMotor.Direction.REVERSE);
        //right1.setDirection(DcMotor.Direction.FORWARD);
        //right2.setDirection(DcMotor.Direction.FORWARD);
        left1.setPower(1.0f);
    }

    @Override
    public void loop() {
        //left1.setPower(0.5f);
    }
}