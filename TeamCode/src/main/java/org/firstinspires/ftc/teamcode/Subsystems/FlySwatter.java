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
    private final DigitalChannel armLimit;
    private final int armLowPosition;
    private final int armMediumPosition;
    private final int armHighPosition;
    private final double armPower;
    private final double initialWristPosition;
    private final double wristMinPosition;
    private final double wristMaxPosition;
    private final double wristPower;
    private final double flapperForwardPower;
    private final double flapperReversePower;

    private double wristPosition;

    private enum ArmPosition {
        LIMIT,
        LOW,
        MEDIUM,
        HIGH,
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

    private enum ArmState {
        STOP,
        LOW,
        MEDIUM,
        HIGH,
        MANUAL_CONTROL,
        LIMITED,
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
            final DigitalChannel armLimit,
            final int armLowPosition,
            final int armMediumPosition,
            final int armHighPosition,
            final double armPower,
            final double initialWristPosition,
            final double wristMinPosition,
            final double wristMaxPosition,
            final double wristPower,
            final double flapperForwardPower,
            final double flapperReversePower
    ) {
        this.arm = arm;
        this.wrist = wrist;
        this.flapper = flapper;
        this.armLimit = armLimit;
        this.armLowPosition = armLowPosition;
        this.armMediumPosition = armMediumPosition;
        this.armHighPosition = armHighPosition;
        this.armPower = armPower;
        this.initialWristPosition = initialWristPosition;
        this.wristMinPosition = wristMinPosition;
        this.wristMaxPosition = wristMaxPosition;
        this.wristPower = wristPower / 100;  // servos need really fine-grained control
        this.flapperForwardPower = flapperForwardPower;
        this.flapperReversePower = flapperReversePower;

        this.armLimit.setMode(DigitalChannel.Mode.INPUT);
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
    
    public void arm(double power) {
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (power < 0 && !armLimit.getState()) {
            armDesiredState = ArmState.LIMITED;
            arm.setPower(0);
        } else {
            armDesiredState = ArmState.MANUAL_CONTROL;
            arm.setPower(power);
        }
    }

    public void armLow() {
        armDesiredState = ArmState.LOW;
        runArmToPosition(armLowPosition);
    }

    public void armMedium() {
        armDesiredState = ArmState.MEDIUM;
        runArmToPosition(armMediumPosition);
    }

    public void armHigh() {
        armDesiredState = ArmState.HIGH;
        runArmToPosition(armHighPosition);
    }

    private void runArmToPosition(int position) {
        arm.setTargetPosition(position);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(armPower);
    }

    public void armStop() {
        armDesiredState = ArmState.STOP;
        arm.setPower(0);

        if (!armLimit.getState()) {
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
        wristPosition = Range.clip(wristPosition + adjustment, wristMinPosition, wristMaxPosition);
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
    
    private ArmPosition getArmPosition() {
        final int armCurrentPosition = arm.getCurrentPosition();
        if (!armLimit.getState()) {
            return ArmPosition.LIMIT.withEncoderCount(armCurrentPosition);
        }
        switch (armDesiredState) {
            case LOW:
                return ArmPosition.LOW.withEncoderCount(armCurrentPosition);
            case MEDIUM:
                return ArmPosition.MEDIUM.withEncoderCount(armCurrentPosition);
            case HIGH:
                return ArmPosition.HIGH.withEncoderCount(armCurrentPosition);
            default:
                return ArmPosition.UNKNOWN.withEncoderCount(armCurrentPosition);
        }
    }

    @Override
    public String toString() {
        return String.format(
                "ArmPosition(%s)%n ArmDesiredState(%s)%n Wrist(%.3f)%n Flapper(%s)%n",
                getArmPosition(), armDesiredState, wrist.getPosition(), flapperState);
    }
}
