package org.firstinspires.ftc.teamcode.RobotConfiguration.FreightFrenzy;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotCoreExtensions.Drivable;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.Drivetrain;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.Initializable;
import org.firstinspires.ftc.teamcode.Subsystems.FlySwatter;

public class FreightFrenzyRobot implements Drivable, Initializable {
    // Components
    private final Drivetrain drivetrain;

    public final BNO055IMU imu;
    public final DcMotor duckSpinner;
    public final FlySwatter flySwatter;

    public FreightFrenzyRobot(HardwareMap hardwareMap) {
        // Drivetrain
        final DcMotor frontLeft = hardwareMap.dcMotor.get("leftf");
        final DcMotor backLeft = hardwareMap.dcMotor.get("leftr");
        final DcMotor frontRight = hardwareMap.dcMotor.get("rightf");
        final DcMotor backRight = hardwareMap.dcMotor.get("rightr");

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

        // vertical wheel thing
        this.duckSpinner = hardwareMap.dcMotor.get("duckSpinner");

        // imu
        this.imu = hardwareMap.get(BNO055IMU.class, "imu");

        //
        this.flySwatter = new FlySwatter(
                hardwareMap.dcMotor.get("arm1"),
                hardwareMap.dcMotor.get("arm2"),
                hardwareMap.servo.get("wrist"),
                hardwareMap.dcMotor.get("flapper"),
                hardwareMap.digitalChannel.get("armLow"),
                hardwareMap.digitalChannel.get("armMiddle"),
                hardwareMap.digitalChannel.get("armHigh"),
                hardwareMap.digitalChannel.get("armPickup"),
                .5,
                .5,
                .5,
                1.0,
                .5  // should still be positive
        );
    }

    @Override
    public void initialize() {
        this.imu.initialize(getImuParameters());
        this.flySwatter.initialize();
    }

    @Override
    public Drivetrain getDrivetrain() {
        return this.drivetrain;
    }

    private BNO055IMU.Parameters getImuParameters() {
        final BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        return parameters;
    }
}
