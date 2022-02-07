package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * We ALWAYS must line up to the two dots furthest from the edges of the field.
 * The robot will always be closest to the middle of the field.
 */
public class ShippingElementFinder {

    private final DistanceSensor frontLeft;
    private final DistanceSensor backLeft;
    private final DistanceSensor frontRight;
    private final DistanceSensor backRight;
    private final double detectableDistanceInches;

    public ShippingElementFinder(
            final double detectableDistanceInches,
            final DistanceSensor frontLeft,
            final DistanceSensor backLeft,
            final DistanceSensor frontRight,
            final DistanceSensor backRight
    ) {
        this.detectableDistanceInches = detectableDistanceInches;
        this.frontLeft = frontLeft;
        this.backLeft = backLeft;
        this.frontRight = frontRight;
        this.backRight = backRight;
    }

    /**
     * Returns the goal position where we should score
     * @return
     */
    public GoalPosition findGoalPosition(final Color color, final Side side) {
        if (side == Side.DUCK_SPINNER) {
            return findGoalPositionNearDuckSpinner(color);
        } else {
            return findGoalPositionNearBarrier(color);
        }
    }

    public enum Color {
        RED,
        BLUE
    }

    public enum Side {
        DUCK_SPINNER,
        BARRIER
    }

    /**
     * These are referencing the tiered goal
     */
    public enum GoalPosition {
        LOW,
        MEDIUM,
        HIGH
    }

    private GoalPosition findGoalPositionNearDuckSpinner(final Color color) {
        if (color == Color.RED) {
            return findGoalPositionUsingSensors(backRight, frontRight);
        } else {
            return findGoalPositionUsingSensors(backLeft, frontLeft);
        }
    }

    private GoalPosition findGoalPositionNearBarrier(final Color color) {
        if (color == Color.RED) {
            return findGoalPositionUsingSensors(backLeft, frontLeft);
        } else {
            return findGoalPositionUsingSensors(backRight, frontRight);
        }
    }

    private GoalPosition findGoalPositionUsingSensors(
            final DistanceSensor lowDetector,
            final DistanceSensor mediumDetector
    ) {
        if (lowDetector.getDistance(DistanceUnit.INCH) < detectableDistanceInches) {
            return GoalPosition.LOW;
        } else if (mediumDetector.getDistance(DistanceUnit.INCH) < detectableDistanceInches) {
            return GoalPosition.MEDIUM;
        } else {
            return GoalPosition.HIGH;
        }
    }

    @Override
    public String toString() {
        return String.format("frontLeft: %.0f%n backLeft: %.0f%n frontRight: %.0f%n backRight: %.0f%n",
                frontLeft.getDistance(DistanceUnit.INCH),
                backLeft.getDistance(DistanceUnit.INCH),
                frontRight.getDistance(DistanceUnit.INCH),
                backRight.getDistance(DistanceUnit.INCH));
    }
}
