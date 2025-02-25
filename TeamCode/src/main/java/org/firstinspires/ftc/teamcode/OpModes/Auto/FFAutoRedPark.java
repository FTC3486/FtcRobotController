package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotConfiguration.FreightFrenzy.FreightFrenzyRobot;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.EncoderAutoDriver;

@Autonomous(group = "Red")
public class FFAutoRedPark extends LinearOpMode {

    @Override
    public void runOpMode() {
        final FreightFrenzyRobot ffRobot = new FreightFrenzyRobot(this.hardwareMap);
        final EncoderAutoDriver encoderAutoDriver = new EncoderAutoDriver(ffRobot, this);
        encoderAutoDriver.setImu(ffRobot.imu);
        ffRobot.initialize();

        waitForStart();

        // DO MOVEMENT

        // move forward into parking thing
        encoderAutoDriver.driveToDistance(15, 0.4);
        telemetry.update();

        ffRobot.flySwatter.armHigh();
        ffRobot.extendi.setPower(1);
        sleep(3000);
        ffRobot.extendi.setPower(0);

        while (opModeIsActive()) {
            telemetry.addData("FlySwatter", ffRobot.flySwatter);
            telemetry.update();
        }
    }
}
