package org.firstinspires.ftc.teamExperiment;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by clint on 12/4/16.
 */

public interface Drivetrain {

    public DcMotor[] getLeftMotors();
    public DcMotor[] getRightMotors();
    public DcMotor[] getFrontMotors();
    public DcMotor[] getRearMotors();

    public Telemetry getTelemetry();
    public BNO055IMU getIMU();

    public DcMotor[] getDriveMotors();

}
