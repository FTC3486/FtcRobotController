package org.firstinspires.ftc.teamcode.RobotConfiguration.UltimateGoal;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotCoreExtensions.ContinuousServo;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.Drivable;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.Drivetrain;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.Initializable;
import org.firstinspires.ftc.teamcode.Subsystems.TapeMeasure;

public class LordOfTheRingsRobot implements Drivable, Initializable {
    // Components
    private final Drivetrain drivetrain;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    public final TapeMeasure tapeMeasure;

    public LordOfTheRingsRobot(HardwareMap hardwareMap) {
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
                .addLeftMotorWithEncoder(backRight)
                .addLeftMotor(frontRight)
                .addRightMotor(frontLeft)
                .addRightMotorWithEncoder(backLeft)
                .setGearRatio(1.2)
                .build();

        tapeMeasure = new TapeMeasure(hardwareMap.crservo.get("tapeMotor"), hardwareMap.servo.get("tapeTilt"), 0.37);
    }


    @Override
    public void initialize() {
        tapeMeasure.initialize();
    }

    @Override
    public Drivetrain getDrivetrain() {
        return this.drivetrain;
    }
}
