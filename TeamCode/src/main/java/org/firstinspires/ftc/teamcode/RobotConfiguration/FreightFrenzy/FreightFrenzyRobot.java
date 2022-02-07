package org.firstinspires.ftc.teamcode.RobotConfiguration.FreightFrenzy;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotCoreExtensions.Drivable;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.Drivetrain;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.Initializable;
import org.firstinspires.ftc.teamcode.Subsystems.FlySwatter;
import org.firstinspires.ftc.teamcode.Subsystems.ShippingElementFinder;

public class FreightFrenzyRobot implements Drivable, Initializable {
    // Components
    private final Drivetrain drivetrain;

    public final BNO055IMU imu;
    public final DcMotor duckSpinner;
    public final FlySwatter flySwatter;
    public final DcMotor extendi;
    public final ShippingElementFinder shippingElementFinder;

    public FreightFrenzyRobot(HardwareMap hardwareMap) {
        // Drivetrain
        final DcMotor frontLeft = hardwareMap.dcMotor.get("leftf");
        final DcMotor backLeft = hardwareMap.dcMotor.get("leftr");
        final DcMotor frontRight = hardwareMap.dcMotor.get("rightf");
        final DcMotor backRight = hardwareMap.dcMotor.get("rightr");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        this.drivetrain = new Drivetrain.Builder()
                .addLeftMotor(frontLeft)
                .addLeftMotorWithEncoder(backLeft)
                .addRightMotor(frontRight)
                .addRightMotorWithEncoder(backRight)
                .setGearRatio(1.2)
                .build();

        // vertical wheel thing
        this.duckSpinner = hardwareMap.dcMotor.get("duckSpinner");

        this.extendi = hardwareMap.dcMotor.get("extendi");
        this.extendi.setDirection(DcMotorSimple.Direction.REVERSE);

        // imu
        this.imu = hardwareMap.get(BNO055IMU.class, "imu");

        this.flySwatter = new FlySwatter(
                hardwareMap.dcMotor.get("arm"),
                hardwareMap.servo.get("wrist"),
                hardwareMap.dcMotor.get("flapper"),
                hardwareMap.digitalChannel.get("armLimit"),
                1000,
                2000,
                5100,
                1.0,
                0.1,
                0.1,
                .8,
                .15,
                1.0,
                .5  // should still be positive
        );

        this.shippingElementFinder = new ShippingElementFinder(
                36,
                hardwareMap.get(DistanceSensor.class, "distance4"),
                hardwareMap.get(DistanceSensor.class, "distance2"),
                hardwareMap.get(DistanceSensor.class, "distance1"),
                hardwareMap.get(DistanceSensor.class, "distance3")
        );
    }

    @Override
    public void initialize() {
        this.extendi.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.extendi.setPower(0);

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
