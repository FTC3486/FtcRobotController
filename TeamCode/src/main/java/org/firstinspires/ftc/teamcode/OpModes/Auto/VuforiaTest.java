package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaSkyStone;
import org.firstinspires.ftc.teamcode.RobotCoreExtensions.VuforiaDriver;

import java.util.Arrays;
import java.util.List;

@TeleOp(name = "Vuforia Testing", group = "Concept")
@Disabled
public class VuforiaTest extends LinearOpMode {

    VuforiaDriver vuforiaDriver;

    @Override
    public void runOpMode() throws InterruptedException {

        vuforiaDriver = new VuforiaDriver(new VuforiaSkyStone(), getTrackableNames(), "AUWvyFD/////AAABmaAT+Xe4Iks6qo/7ra3yndh8HtUQLcEHWOxW1ZN74TVmgwOIg6aPYTrI0zh6peSAzci+oaRE3SteoHRMP3gLxfzjW98ja2voVSq1M1wLw3jFhVyd6Yv/RzYQ+HWQi+rpDkyLUv7GKOj+vBmGwxW8oyrazADHJ0a0VNoNcX1vPZhN8bwN3xRUzlG0g35012sY2CehU613sQw7GNOqmCVAEfVHBrVN6EdcYWIj5i1YpsXWb899AigohjtDu83oO1MeZQCWTinNHHMr6KUCX+Dn1wGvbJBKJrWA5liHUBaUOUScEylDa6Zp5ulY6fWr2YUszVRiTwR8WeqoATNTcbarq6psjT/sbmeX42nE/MKkRA9F");
        vuforiaDriver.initialize(VuforiaLocalizer.CameraDirection.BACK, new Position(DistanceUnit.INCH, 4, 8, 0, 0), new Orientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, 90, -90, 0, 0));

        waitForStart();

        while (opModeIsActive()) {
            //telemetry.addData("Stone Position", vuforiaDriver.getTrackableLocation(VuforiaSkyStone.TRACKABLE_NAMES[0]));

            try {
                telemetry.addData("Robot Position", vuforiaDriver.getRobotLocation());
                telemetry.update();
            } catch (VuforiaDriver.PositionCalculationException e) {
                telemetry.addData("Failed to get Robot position", e.getMessage());
                telemetry.update();
            }
        }
    }

    /**
     * Get all trackables EXCEPT for the "STONE_TARGET" trackable, which doesn't have a single known position
     *
     * @return
     */
    private List<String> getTrackableNames() {
        return Arrays.asList(VuforiaSkyStone.TRACKABLE_NAMES).subList(1, VuforiaSkyStone.TRACKABLE_NAMES.length);
    }
}
