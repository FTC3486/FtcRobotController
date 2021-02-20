package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotCoreExtensions.Initializable;

public class VelocityMotor implements Initializable {
    private final DcMotorEx motor;
    private final double defaultPower;

    private enum VelocityMotorState {
        RUNNING,
        REVERSING,
        STOPPED,
    }

    private VelocityMotorState velocityMotorState;

    public VelocityMotor(DcMotorEx motor, double defaultPower) {
        this.motor = motor;
        this.defaultPower = defaultPower;
    }

    @Override
    public void initialize() {
        stop();
    }

    public void run() {
        run(defaultPower);
    }

    public void run(double power) {
        motor.setPower(power);
        velocityMotorState = VelocityMotorState.RUNNING;
    }

    public double getVelocity() {
        return motor.getVelocity();
    }

    public void setVelocity(double velocity) {
        motor.setVelocity(velocity);
    }

    public void stop() {
        motor.setPower(0);
        velocityMotorState = VelocityMotorState.STOPPED;
    }

    public void reverse() {
        reverse(-defaultPower);
    }

    public void reverse(double power) {
        motor.setPower(power);
        velocityMotorState = VelocityMotorState.REVERSING;
    }



    @Override
    public String toString() {
        switch (velocityMotorState) {
            case RUNNING:
                return "Running";

            case REVERSING:
                return "Running in Reverse";

            case STOPPED:
                return "Stopped";

            default:
                return "Unknown";
        }
    }

}