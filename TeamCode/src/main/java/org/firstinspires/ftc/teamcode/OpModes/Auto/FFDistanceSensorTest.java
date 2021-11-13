package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotConfiguration.FreightFrenzy.FreightFrenzyRobot;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.EncoderAutoDriver;

@Autonomous
public class FFDistanceSensorTest extends LinearOpMode{
    private FreightFrenzyRobot ffRobot;
    EncoderAutoDriver encoderAutoDriver;


    DistanceSensor distance1;
    DistanceSensor distance2;

    @Override
    public void runOpMode() {
        //setup
        ffRobot = new FreightFrenzyRobot(this.hardwareMap, this);
        encoderAutoDriver = new EncoderAutoDriver(ffRobot, this);
        telemetry.addData("Test", ffRobot.getDrivetrain().getLeftEncoderCount());
        ffRobot.getDrivetrain().resetMotorEncoders();
        ffRobot.initialize();
        ffRobot.bucketboi.close();
        encoderAutoDriver.setImu(ffRobot.imu);
        waitForStart();


        while(opModeIsActive()){
            telemetry.addData("LeftSensor: ", ffRobot.distance_left());
            telemetry.addData("RightSensor: ", ffRobot.distance_right());
            telemetry.update();

        }
    }
}