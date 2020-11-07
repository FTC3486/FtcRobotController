package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotCoreExtensions.Initializable;

public class OpenCloseServo implements Initializable {
    private final Servo servo;
    private final double initializedPosition;
    private final double openedPosition;
    private final double closedPosition;

    private enum OpenCloseServoState {
        INITIALIZED,
        OPENED,
        CLOSED,
    }

    private OpenCloseServoState openCloseServoState;

    public OpenCloseServo(
            final Servo servo,
            double initializedPosition,
            double openedPosition,
            double closedPosition) {
        this.servo = servo;
        this.initializedPosition = initializedPosition;
        this.openedPosition = openedPosition;
        this.closedPosition = closedPosition;
    }

    @Override
    public void initialize() {
        servo.setPosition(initializedPosition);
        openCloseServoState = OpenCloseServoState.INITIALIZED;
    }

    public void open() {
        servo.setPosition(openedPosition);
        openCloseServoState = OpenCloseServoState.OPENED;
    }

    public void close() {
        servo.setPosition(closedPosition);
        openCloseServoState = OpenCloseServoState.CLOSED;
    }

    @Override
    public String toString() {
        switch (openCloseServoState) {
            case INITIALIZED:
                return "Initialized";

            case OPENED:
                return "Opened";

            case CLOSED:
                return "Closed";

            default:
                return "Unknown";
        }
    }
}