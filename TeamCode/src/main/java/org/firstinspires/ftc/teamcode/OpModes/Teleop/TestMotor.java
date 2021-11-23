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
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;

@TeleOp(name = "Test Motor", group = "Teleop2021")
@Disabled
public class TestMotor extends OpMode {
    //Declare parts of the robot that will be used by this Teleop
    ExpansionHubMotor leftf, rightf, rightr,leftr;

    @Override
    public void init() {
        DcMotor leftf_, leftr_, rightf_, rightr_;

        leftf_ = hardwareMap.dcMotor.get("leftf");
        rightf_ = hardwareMap.dcMotor.get("rightf");
        leftr_ = hardwareMap.dcMotor.get("leftr");
        rightr_ = hardwareMap.dcMotor.get("rightr");

        leftf = (ExpansionHubMotor) hardwareMap.dcMotor.get("leftf");
        rightf = (ExpansionHubMotor) hardwareMap.dcMotor.get("rightf");
        leftr = (ExpansionHubMotor) hardwareMap.dcMotor.get("leftr");
        rightr = (ExpansionHubMotor) hardwareMap.dcMotor.get("rightr");

        leftf_.setDirection(DcMotor.Direction.REVERSE);
        rightf_.setDirection(DcMotor.Direction.REVERSE);
        leftr_.setDirection(DcMotor.Direction.REVERSE);
        rightr_.setDirection(DcMotor.Direction.REVERSE);

        //waitForStart();

        leftf_.setPower(0.5f);
        rightf_.setPower(0.5f);
        leftr_.setPower(0.5f);
        rightr_.setPower(0.5f);
    }

    @Override
    public void loop() {
        telemetry.addData("Left front", leftf.getVelocity());
        telemetry.addData("Right front", rightf.getVelocity());
        telemetry.addData("Left Rear", leftr.getVelocity());
        telemetry.addData("Right Rear", rightr.getVelocity());
        telemetry.update();
    }
}