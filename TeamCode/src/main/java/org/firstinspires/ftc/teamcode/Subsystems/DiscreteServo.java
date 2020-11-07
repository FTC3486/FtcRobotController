package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotCoreExtensions.Initializable;

import java.util.HashMap;
import java.util.Map;

public class DiscreteServo<D extends DiscreteServo.DiscreteServoPosition> implements Initializable {
    private final Servo servo;
    private final Map<String, D> servoPositions;
    private final D initialPosition;

    private D discreteServoPosition;

    public DiscreteServo(
            final Servo servo,
            final D initialPosition,
            final D[] servoPositions
    ) {
        this.servo = servo;
        this.servoPositions = getPositionsMap(servoPositions);
        this.initialPosition = initialPosition;
    }

    public interface DiscreteServoPosition {
        String name();
        double getPosition();
    }

    private Map<String, D> getPositionsMap(D[] servoPositions) {
        final Map<String, D> positionsMap = new HashMap<>();
        for (D position : servoPositions) {
            positionsMap.put(position.name(), position);
        }
        return positionsMap;
    }

    @Override
    public void initialize() {
        setPosition(initialPosition);
    }

    public void setPosition(D position) {
        final D newPosition = servoPositions.get(position.name());
        servo.setPosition(newPosition.getPosition());
        this.discreteServoPosition = newPosition;
    }

    public D getDesiredPosition() {
        return discreteServoPosition;
    }

    @Override
    public String toString() {
        return discreteServoPosition.name();
    }
}
