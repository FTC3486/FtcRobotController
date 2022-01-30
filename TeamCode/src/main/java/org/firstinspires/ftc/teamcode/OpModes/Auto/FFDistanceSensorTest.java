package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotConfiguration.FreightFrenzy.FreightFrenzyRobot;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.EncoderAutoDriver;
@Autonomous
public class FFDistanceSensorTest extends LinearOpMode{
    private FreightFrenzyRobot ffRobot;
    EncoderAutoDriver encoderAutoDriver;

    @Override
    public void runOpMode() {
        //setup
        ffRobot = new FreightFrenzyRobot(this.hardwareMap);
        encoderAutoDriver = new EncoderAutoDriver(ffRobot, this);
        telemetry.addData("Test", ffRobot.getDrivetrain().getLeftEncoderCount());
        ffRobot.getDrivetrain().resetMotorEncoders();
        ffRobot.initialize();

        encoderAutoDriver.setImu(ffRobot.imu);
        waitForStart();

        ffRobot.flySwatter.armLow();

        while(opModeIsActive()) {
         // telemetry.addData("LeftSensor: ", ffRobot.distance2.getDistance(DistanceUnit.INCH));
           //telemetry.addData("RightSensor: ", ffRobot.distance1.getDistance(DistanceUnit.INCH));
            telemetry.addData("FlySwatter", ffRobot.flySwatter);
            telemetry.update();
        }
    }
}