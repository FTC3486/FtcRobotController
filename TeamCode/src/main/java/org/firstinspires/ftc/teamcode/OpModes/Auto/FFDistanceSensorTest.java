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
import org.firstinspires.ftc.teamcode.Subsystems.ShippingElementFinder;

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

        //ffRobot.flySwatter.armLow();

        /*
        3 positions on the field
        2 distance sensors. They will read A or B
        If they read distance A or B, we go to A or B.
        If they read nothing we go to C
         */

        while(opModeIsActive()) {
            telemetry.addData("Red/Duck Goal", ffRobot.shippingElementFinder.findGoalPosition(ShippingElementFinder.Color.RED, ShippingElementFinder.Side.DUCK_SPINNER));
            telemetry.addData("Red/Barrier Goal", ffRobot.shippingElementFinder.findGoalPosition(ShippingElementFinder.Color.RED, ShippingElementFinder.Side.BARRIER));
            telemetry.addData("Blue/Duck Goa:", ffRobot.shippingElementFinder.findGoalPosition(ShippingElementFinder.Color.BLUE, ShippingElementFinder.Side.DUCK_SPINNER));
            telemetry.addData("Blue/Barrier Goal", ffRobot.shippingElementFinder.findGoalPosition(ShippingElementFinder.Color.BLUE, ShippingElementFinder.Side.BARRIER));

            telemetry.addData("ShippingElementFinder", ffRobot.shippingElementFinder);

            telemetry.update();
        }
    }
}