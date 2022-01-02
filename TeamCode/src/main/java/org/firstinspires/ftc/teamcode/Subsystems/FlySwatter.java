package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotCoreExtensions.Initializable;

public class FlySwatter implements Initializable {
    // These motors are physically coupled and must be turned simultaneously
    private final DcMotor arm1;
    private final DcMotor arm2;

    private final Servo wrist;
    private final DcMotor flapper;

    private final DigitalChannel armHigh;
    private final DigitalChannel armMiddle;
    private final DigitalChannel armLow;
    private final DigitalChannel armPickup;
    private final double armPower;
    private final double initialWristPosition;
    private final double flapperForwardPower;
    private final double flapperReversePower;

    private double wristPosition;
    private double wristPower;

    private enum ArmPosition {
        LOW,
        MIDDLE,
        HIGH,
        PICKUP,
        UNKNOWN
    }
    private ArmPosition armLastKnownPosition = ArmPosition.UNKNOWN;

    private enum ArmState {
        STOP,
        LOW,
        MIDDLE,
        HIGH,
        PICKUP,
        MANUAL_CONTROL,
        UNKNOWN
    }
    private ArmState armDesiredState = ArmState.UNKNOWN;

    private enum FlapperState {
        STOP,
        FORWARD,
        REVERSE
    }

    private FlapperState flapperState;

    public FlySwatter(
            final DcMotor arm1,
            final DcMotor arm2,
            final Servo wrist,
            final DcMotor flapper,
            final DigitalChannel armLow,
            final DigitalChannel armMiddle,
            final DigitalChannel armHigh,
            final DigitalChannel armPickup,
            final double armPower,
            final double initialWristPosition,
            double wristPower,
            final double flapperForwardPower,
            final double flapperReversePower
    ) {
        this.arm1 = arm1;
        this.arm2 = arm2;
        this.wrist = wrist;
        this.flapper = flapper;
        this.armLow = armLow;
        this.armMiddle = armMiddle;
        this.armHigh = armHigh;
        this.armPickup = armPickup;
        this.armPower = armPower;
        this.initialWristPosition = initialWristPosition;
        this.wristPower = wristPower / 100;  // servos need really fine-grained control
        this.flapperForwardPower = flapperForwardPower;
        this.flapperReversePower = flapperReversePower;

        this.armLow.setMode(DigitalChannel.Mode.INPUT);
        this.armMiddle.setMode(DigitalChannel.Mode.INPUT);
        this.armHigh.setMode(DigitalChannel.Mode.INPUT);
        this.armPickup.setMode(DigitalChannel.Mode.INPUT);
    }

    @Override
    public void initialize() {
        
        // drive up to any switch
        while (getArmStateActual() == ArmPosition.UNKNOWN) {
            arm(armPower);
        }
        armStop();
        
        // Maybe move to low?
        
        wrist.setPosition(initialWristPosition);
        wristPosition = initialWristPosition;
    }

    public void armLow() {
        armDesiredState = ArmState.LOW;
        if (getArmStateActual() == ArmPosition.LOW) {
            setArmPower(0);
        } else {
            switch (armLastKnownPosition) {
                case LOW:
                    // Maybe we passed the limit switch? Head up to Middle
                    setArmPower(armPower);
                    break;
                case MIDDLE:
                case HIGH:
                case PICKUP:
                case UNKNOWN:
                    setArmPower(-armPower);
                    break;
            }
        }
    }

    public void armMiddle() {
        armDesiredState = ArmState.MIDDLE;
        if (getArmStateActual() == ArmPosition.MIDDLE) {
            setArmPower(0);
        } else {
            switch (armLastKnownPosition) {
                case LOW:
                    // Maybe we passed the limit switch? Head up to Middle
                    setArmPower(armPower);
                    break;
                case MIDDLE:
                case HIGH:
                case PICKUP:
                case UNKNOWN:
                    setArmPower(-armPower);
                    break;
            }
        }
    }

    public void armHigh() {
        armDesiredState = ArmState.HIGH;
        if (getArmStateActual() == ArmPosition.HIGH) {
            setArmPower(0);
        } else {
            switch (armLastKnownPosition) {
                case LOW:
                case MIDDLE:
                case UNKNOWN:
                    setArmPower(armPower);
                    break;
                case HIGH:
                    // We drive down when last-seen is high because middle is closer to high than pickup
                case PICKUP:
                    setArmPower(-armPower);
                    break;
            }
        }
    }
    
    public void armPickup() {
        armDesiredState = ArmState.PICKUP;
        if (getArmStateActual() == ArmPosition.PICKUP) {
            setArmPower(0);
        } else {
            switch (armLastKnownPosition) {
                case LOW:
                case MIDDLE:
                case HIGH:
                case UNKNOWN:
                    setArmPower(armPower);
                    break;
                case PICKUP:
                    setArmPower(-armPower);
                    break;
            }
        }
    }
    
    public void arm(double power) {
        armDesiredState = ArmState.MANUAL_CONTROL;
        setArmPower(power);
    }

    public void armStop() {
        armDesiredState = ArmState.STOP;
        setArmPower(0);
    }

    private void setArmPower(double power) {
        arm1.setPower(power);
        arm2.setPower(power);
    }

    public void wristUp() {
        setWristPosition(wristPower);
    }

    public void wristDown() {
        setWristPosition(-wristPower);
    }

    private void setWristPosition(double adjustment) {
        wristPosition = Range.clip(wristPosition + adjustment, 0.1, 0.9);
        wrist.setPosition(wristPosition);
    }
    
    public void flapperForward() {
        flapper.setPower(flapperForwardPower);
        flapperState = FlapperState.FORWARD;
    }
    
    public void flapperReverse() {
        flapper.setPower(-flapperReversePower);
        flapperState = FlapperState.REVERSE;
    }
    
    public void flapperStop() {
        flapper.setPower(0);
        flapperState = FlapperState.STOP;
    }
    
    private ArmPosition getArmStateActual() {
        if (!armLow.getState()) {
            armLastKnownPosition = ArmPosition.LOW;
            return ArmPosition.LOW;
        }
        if (!armMiddle.getState()) {
            armLastKnownPosition = ArmPosition.MIDDLE;
            return ArmPosition.MIDDLE;
        }
        if (!armHigh.getState()) {
            armLastKnownPosition = ArmPosition.HIGH;
            return ArmPosition.HIGH;
        }
        if (!armPickup.getState()) {
            armLastKnownPosition = ArmPosition.PICKUP;
            return ArmPosition.PICKUP;
        }
        return ArmPosition.UNKNOWN;
    }

    @Override
    public String toString() {
        return String.format(
                "ArmStateActual(%s)%n ArmDesiredState(%s)%n ArmLastKnownPosition(%s)%n Wrist(%.3f)%n Flapper(%s)%n",
                getArmStateActual(), armDesiredState, armLastKnownPosition, wrist.getPosition(), flapperState);
    }
}
