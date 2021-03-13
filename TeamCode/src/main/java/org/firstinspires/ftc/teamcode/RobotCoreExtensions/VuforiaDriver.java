package org.firstinspires.ftc.teamcode.RobotCoreExtensions;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

public class VuforiaDriver implements AutoCloseable {
    private final VuforiaBase vuforia;
    private final List<String> trackableNames;
    private final String vuforiaLicenseKey;

    public VuforiaDriver(
            final VuforiaBase vuforia,
            final List<String> trackableNames,
            final String vuforiaLicenseKey
    ) {
        this.vuforia = vuforia;
        this.trackableNames = trackableNames;
        this.vuforiaLicenseKey = vuforiaLicenseKey;
    }

    public void initialize(
            final VuforiaLocalizer.CameraDirection cameraDirection,
            final Position cameraPosition,
            final Orientation cameraOrientation
    ) {
        final Position cameraPositionMm = cameraPosition.toUnit(DistanceUnit.MM);
        final Orientation cameraOrientationDeg = cameraOrientation
                .toAngleUnit(AngleUnit.DEGREES)
                .toAxesOrder(AxesOrder.XYZ)
                .toAxesReference(AxesReference.EXTRINSIC);

        vuforia.initialize(
                vuforiaLicenseKey,
                cameraDirection,
                true,
                true,
                VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES,
                (float) cameraPositionMm.x,
                (float) cameraPositionMm.y,
                (float) cameraPositionMm.z,
                cameraOrientationDeg.firstAngle,
                cameraOrientationDeg.secondAngle,
                cameraOrientationDeg.thirdAngle,
                true
        );

        vuforia.activate();
    }

    /**
     * Calculate a single pose (position and orientation) from all tracking results.
     * <p>
     * When we received more than one tracking result claiming to have updated information about the
     * location of the robot, we should take care not to randomly select just one of these results
     * and run with it. The example code does this, and it could lead to a poor result.
     * <p>
     * Weighting is performed by sorting the results by distance from the robot and visibility. The
     * assumptions we are making are these:
     * <p>
     * 1. Visible markers will have more accurate readings than markers no longer visible.
     * 2. The closer a marker is to the robot, the more accurate its reading will be.
     *
     * @return The location of the robot (position and orientation)
     */
    public Location getRobotLocation() throws PositionCalculationException {
        List<RobotPoseAndAssociatedMarker> robotPoseData = new ArrayList<>();
        for (String name : trackableNames) {
            RobotPoseAndAssociatedMarker robotPoseAndAssociatedMarker = new RobotPoseAndAssociatedMarker(vuforia.track(name), vuforia.trackPose(name));
            if (robotPoseAndAssociatedMarker.robotPose.matrix != null && robotPoseAndAssociatedMarker.markerPose.matrix != null) {
                robotPoseData.add(robotPoseAndAssociatedMarker);
            }
        }

        Collections.sort(robotPoseData, (o1, o2) -> Float.compare(
                o1.markerPose.matrix.getTranslation().magnitude(),
                o2.markerPose.matrix.getTranslation().magnitude()));
        Collections.sort(robotPoseData, (o1, o2) -> Boolean.compare(
                o1.robotPose.isVisible,
                o2.robotPose.isVisible));
        Collections.reverse(robotPoseData);

        if (robotPoseData.isEmpty()) {
            throw new PositionCalculationException("After filtering for robot poses with " +
                    "available position data, there were no poses available from which a " +
                    "position could be calculated");
        }

        final float scalingFactor = 0.8F;
        final float geometricSumReciprocal = (float)
                ((1 - scalingFactor) / (1 - Math.pow(scalingFactor, robotPoseData.size())));

        boolean seen = false;
        OpenGLMatrix acc = null;
        for (RobotPoseAndAssociatedMarker pose : robotPoseData) {
            OpenGLMatrix matrix = pose.robotPose.matrix;
            if (!seen) {
                seen = true;
                acc = matrix;
            } else {
                acc = new OpenGLMatrix(acc.scaled(0.8F).added(matrix));
            }
        }

        if (acc == null) {
            throw new PositionCalculationException("After performing a weighted average, there " +
                    "were no poses available from which a position could be calculated");
        }


        final VectorF positionVector = acc.scaled(geometricSumReciprocal).getTranslation();
        final Position position = new Position(DistanceUnit.MM, positionVector.get(0), positionVector.get(1), positionVector.get(2), 0);
        final Orientation orientation = Orientation.getOrientation(acc, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        return new Location(position, orientation);
    }

    public Location getTrackableLocation(String trackableName) throws PositionCalculationException {

            final VuforiaBase.TrackingResults trackingResults = vuforia.trackPose(trackableName);
            if (trackingResults != null) {
                final VectorF positionVector = trackingResults.matrix.getTranslation();
                final Position position = new Position(DistanceUnit.MM, positionVector.get(0), positionVector.get(1), positionVector.get(2), 0);
                final Orientation orientation = Orientation.getOrientation(trackingResults.matrix, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                return new Location(position, orientation);
            } else {
            throw new PositionCalculationException("Could not detect item");
        }


    }

    public boolean isVisible(String trackableName) {
        final VuforiaBase.TrackingResults trackingResults = vuforia.trackPose(trackableName);
        return trackingResults.isVisible;
    }

    public static class Location {
        final Position position;
        final Orientation orientation;

        public Location(Position position, Orientation orientation) {
            this.position = position;
            this.orientation = orientation;
        }

        public Position getPosition() {
            return position;
        }

        public Orientation getOrientation() {
            return orientation;
        }

        @Override
        public String toString() {
            final String orientationString = String.format(Locale.getDefault(), "{%.0f %.0f %.0f}", this.orientation.firstAngle, this.orientation.secondAngle, this.orientation.thirdAngle);
            return "Location{" +
                    "position=" + position.toUnit(DistanceUnit.INCH) +
                    ", orientation=" + orientationString +
                    '}';
        }
    }

    /**
     * Container class to store a robot pose (the robot's location on the field) with the marker
     * that generated this location and its pose, which is relative to the robot and not the field.
     */
    private static class RobotPoseAndAssociatedMarker {
        final VuforiaBase.TrackingResults robotPose;
        final VuforiaBase.TrackingResults markerPose;

        RobotPoseAndAssociatedMarker(
                final VuforiaBase.TrackingResults robotPose,
                final VuforiaBase.TrackingResults markerPose
        ) {
            this.robotPose = robotPose;
            this.markerPose = markerPose;
        }
    }

    /**
     * To be thrown when the Vuforia Driver isn't capable of calculating the robot's position.
     */
    public static class PositionCalculationException extends Exception {
        public PositionCalculationException(String message) {
            super(message);
        }
    }

    @Override
    public void close() {
        vuforia.close();
    }
}
