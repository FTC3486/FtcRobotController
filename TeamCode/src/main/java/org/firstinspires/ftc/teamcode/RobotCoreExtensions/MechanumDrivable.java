package org.firstinspires.ftc.teamcode.RobotCoreExtensions;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * If you want your robot to be Drivable, you must implement this interface
 */
public interface MechanumDrivable extends Drivable {
    public DcMotor getFrontLeftMotor();
    public DcMotor getFrontRightMotor();
    public DcMotor getBackLeftMotor();
    public DcMotor getBackRightMotor();
}


