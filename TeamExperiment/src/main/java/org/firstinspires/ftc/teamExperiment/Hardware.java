package org.firstinspires.ftc.teamExperiment;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by clint on 12/2/16.
 */

public abstract class Hardware implements Drivetrain {


    DcMotor frontLeftMotor = null;
    DcMotor frontRightMotor = null;
    DcMotor rearLeftMotor = null;
    DcMotor rearRightMotor = null;

    DcMotor leftShooterMotor = null;
    DcMotor rightShooterMotor = null;


    BNO055IMU imu;

    Servo servo = null;


    DcMotor[] frontMotors = null;
    DcMotor[] rearMotors = null;
    DcMotor[] leftMotors = null;
    DcMotor[] rightMotors = null;
    DcMotor[] driveMotors = null;
    DcMotor[] shooterMotors = null;

    Telemetry telemetry = null;


    public void initHardware(HardwareMap hardwareMap,Telemetry telemetry) {
        initDriveMotors(hardwareMap);
        initShooterMotors(hardwareMap);
        initServo(hardwareMap);
        initIMU(hardwareMap);
        this.telemetry = telemetry;
    }


    public void initDriveMotors(HardwareMap hardwareMap) {

        frontLeftMotor = hardwareMap.dcMotor.get("");
        frontRightMotor = hardwareMap.dcMotor.get("");
        rearLeftMotor = hardwareMap.dcMotor.get("");
        rearRightMotor = hardwareMap.dcMotor.get("");


        frontMotors = new DcMotor[]{frontLeftMotor, frontRightMotor};
        rearMotors = new DcMotor[]{rearLeftMotor, rearRightMotor};
        leftMotors = new DcMotor[]{frontLeftMotor, rearLeftMotor};
        rightMotors = new DcMotor[]{frontRightMotor, rearRightMotor};
        driveMotors = new DcMotor[]{frontRightMotor, rearRightMotor, frontLeftMotor, rearLeftMotor};
        shooterMotors = new DcMotor[]{leftShooterMotor, rightShooterMotor};

        DrivetrainFunctions.setMotorDirection(DcMotor.Direction.REVERSE, rightMotors);
        DrivetrainFunctions.setMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, driveMotors);
    }

    public void initShooterMotors(HardwareMap hardwareMap) {
        leftShooterMotor = hardwareMap.dcMotor.get("");
        rightShooterMotor = hardwareMap.dcMotor.get("");
        shooterMotors = new DcMotor[]{leftShooterMotor, rightShooterMotor};
    }

    public void initIMU(HardwareMap hardwareMap) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void initServo(HardwareMap hardwareMap) {
        servo = hardwareMap.servo.get("");
    }


    @Override
    public DcMotor[] getLeftMotors() {
        return leftMotors.clone();
    }

    @Override
    public DcMotor[] getRightMotors() {
        return rightMotors.clone();
    }

    @Override
    public DcMotor[] getFrontMotors() {
        return frontMotors.clone();
    }

    @Override
    public DcMotor[] getRearMotors() {
        return rearMotors.clone();
    }

    @Override
    public Telemetry getTelemetry() {
        return telemetry;
    }

    @Override
    public BNO055IMU getIMU() {
        return imu;
    }

    @Override
    public DcMotor[] getDriveMotors() {
        return driveMotors.clone();
    }
}
