package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotCoreExtensions.Initializable;

public class FlySwatter implements Initializable {
    private final DcMotor arm;
    private final Servo wrist;
    private final DcMotor flapper;
    private final DigitalChannel armHigh;
    private final DigitalChannel armPickup;
    private final long armMaxEncoderCounts;
    private final long armEncoderCountsPastPickup;
    private final double armPower;
    private final double initialWristPosition;
    private final double wristPower;
    private final double flapperForwardPower;
    private final double flapperReversePower;

    private double wristPosition;

    private enum ArmPosition {
        HIGH,
        PICKUP,
        MAX_LIMIT,
        MIN_LIMIT,
        UNKNOWN;

        private int encoderCount;

        public ArmPosition withEncoderCount(int encoderCount) {
            this.encoderCount = encoderCount;
            return this;
        }

        @Override
        public String toString() {
            return String.format("position=%s encoderCount=%d", name(), encoderCount);
        }
    }
    private ArmPosition armLastKnownPosition = ArmPosition.UNKNOWN;

    private enum ArmState {
        STOP,
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
            final DcMotor arm,
            final Servo wrist,
            final DcMotor flapper,
            final DigitalChannel armHigh,
            final DigitalChannel armPickup,
            final long armMaxEncoderCounts,
            final long armEncoderCountsPastPickup,
            final double armPower,
            final double initialWristPosition,
            final double wristPower,
            final double flapperForwardPower,
            final double flapperReversePower
    ) {
        this.arm = arm;
        this.wrist = wrist;
        this.flapper = flapper;
        this.armMaxEncoderCounts = armMaxEncoderCounts;
        this.armHigh = armHigh;
        this.armPickup = armPickup;
        this.armEncoderCountsPastPickup = armEncoderCountsPastPickup;
        this.armPower = armPower;
        this.initialWristPosition = initialWristPosition;
        this.wristPower = wristPower / 100;  // servos need really fine-grained control
        this.flapperForwardPower = flapperForwardPower;
        this.flapperReversePower = flapperReversePower;

        this.armHigh.setMode(DigitalChannel.Mode.INPUT);
        this.armPickup.setMode(DigitalChannel.Mode.INPUT);
    }

    @Override
    public void initialize() {
        // reset arm encoder
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // move wrist to initial position
        wrist.setPosition(initialWristPosition);
        wristPosition = initialWristPosition;
    }

    public void armHigh() {
        armDesiredState = ArmState.HIGH;
        if (getArmStateActual() == ArmPosition.HIGH) {
            arm.setPower(0);
        } else {
            switch (armLastKnownPosition) {
                case MAX_LIMIT:
                case MIN_LIMIT:
                    armStop();
                    break;
                case UNKNOWN:
                    arm.setPower(armPower);
                    break;
                case HIGH:
                    // We drive down when last-seen is high because middle is closer to high than pickup
                case PICKUP:
                    arm.setPower(-armPower);
                    break;
            }
        }
    }
    
    public void armPickup() {
        armDesiredState = ArmState.PICKUP;
        if (getArmStateActual() == ArmPosition.PICKUP) {
            arm.setPower(0);
        } else {
            switch (armLastKnownPosition) {
                case MAX_LIMIT:
                case MIN_LIMIT:
                    armStop();
                    break;
                case HIGH:
                case UNKNOWN:
                    arm.setPower(armPower);
                    break;
                case PICKUP:
                    arm.setPower(-armPower);
                    break;
            }
        }
    }
    
    public void arm(double power) {
        armDesiredState = ArmState.MANUAL_CONTROL;
        arm.setPower(power);
    }

    public void armStop() {
        armDesiredState = ArmState.STOP;
        arm.setPower(0);

        if (!armPickup.getState()) {
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
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
        final int armCurrentPosition = arm.getCurrentPosition();
        if (!armHigh.getState()) {
            armLastKnownPosition = ArmPosition.HIGH.withEncoderCount(armCurrentPosition);
            return armLastKnownPosition;
        }
        if (!armPickup.getState()) {
            armLastKnownPosition = ArmPosition.PICKUP.withEncoderCount(armCurrentPosition);
            return armLastKnownPosition;
        }
        if (armCurrentPosition <= -armEncoderCountsPastPickup) {
            armLastKnownPosition = ArmPosition.MIN_LIMIT.withEncoderCount(armCurrentPosition);
            return armLastKnownPosition;
        }
        if (armCurrentPosition >= armMaxEncoderCounts) {
            armLastKnownPosition = ArmPosition.MAX_LIMIT.withEncoderCount(armCurrentPosition);
            return armLastKnownPosition;
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
