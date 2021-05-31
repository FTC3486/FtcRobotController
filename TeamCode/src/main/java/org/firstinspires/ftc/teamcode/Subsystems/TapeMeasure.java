package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotCoreExtensions.Initializable;

/**
 * Created by Matthew on 1/16/2016.
 */
public class TapeMeasure implements Initializable {
    final private CRServo tapeMotor;
    final private Servo tapeTilt;
    private final double initialTiltPosition;

    private enum tapeMotorEnum {EXTEND, RETRACT, STOP}

    private tapeMotorEnum tapeMotorState = tapeMotorEnum.STOP;

    enum tapeTiltEnum {UP, DOWN, STOP}

    private tapeTiltEnum tapeTiltState = tapeTiltEnum.STOP;

    public TapeMeasure(CRServo tapeMotor, Servo tapeTilt, double initialTiltPosition) {
        this.tapeMotor = tapeMotor;
        this.tapeTilt = tapeTilt;
        this.initialTiltPosition = initialTiltPosition;
    }

    @Override
    public void initialize() {
        tapeMotor.setPower(0);
        tapeTilt.setPosition(initialTiltPosition);
    }

    public void extendTapeMeasure() {
        tapeMotor.setPower(1.0);
        tapeMotorState = tapeMotorEnum.EXTEND;
    }

    public void retractTapeMeasure() {
        tapeMotor.setPower(-1.0);
        tapeMotorState = tapeMotorEnum.RETRACT;
    }

    public void stopTapeMeasure() {
        tapeMotor.setPower(0.0);
        tapeMotorState = tapeMotorEnum.STOP;
    }

    public void tiltDown() {
        tapeTilt.setPosition(tapeTilt.getPosition() + 0.0002);
        tapeTiltState = tapeTiltEnum.DOWN;
    }

    public void tiltUp() {
        tapeTilt.setPosition(tapeTilt.getPosition() - 0.0002);
        tapeTiltState = tapeTiltEnum.UP;
    }

    public void stopTilt() {
        tapeTiltState = tapeTiltEnum.STOP;
    }

    @Override
    public String toString() {
        String returnString = "TapeMotor ";

        switch (tapeMotorState) {
            case EXTEND:
                returnString += "EXTEND";
                break;

            case RETRACT:
                returnString += "RETRACT";
                break;

            case STOP:
                returnString += "STOP";
                break;
        }

        returnString += "\nTapeTilt ";

        switch (tapeTiltState) {
            case UP:
                returnString += "UP";
                break;

            case DOWN:
                returnString += "DOWN";
                break;

            case STOP:
                returnString += "STOP";
                break;
        }

        returnString += String.format("\nTapeTilt{%.4f}", tapeTilt.getPosition());

        return returnString;
    }
}