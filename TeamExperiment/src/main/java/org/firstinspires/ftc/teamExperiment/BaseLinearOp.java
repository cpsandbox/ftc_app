package org.firstinspires.ftc.teamExperiment;

import android.support.annotation.NonNull;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Arrays;

/**
 * Created by clint on 12/2/16.
 */

public abstract class BaseLinearOp extends LinearOpMode {
    protected static final int TICS_PER_REV =1120;
    protected static final double WHEEL_DIAMETER_IN = 9.325/Math.PI;
    //protected static final double WHEEL_DIAMETER_IN = 12.4/Math.PI; mechanum
    //protected static final double WHEEL_CIRCUMFERENCE_MM = 314.96; mechanum
    protected static final double WHEEL_CIRCUMFERENCE_MM = 236.855;

    protected static final double MAX_TURNING_POWER = .5;
    protected static final double MIN_TURNING_POWER = .05;

    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor rearLeftMotor = null;



    private DcMotor rearRightMotor = null;

    private DcMotor leftShooterMotor = null;
    private DcMotor rightShooterMotor = null;


    protected BNO055IMU imu;

    protected Servo servo = null;


    private DcMotor[] frontMotors = null;
    private DcMotor[] rearMotors = null;
    private DcMotor[] leftMotors = null;
    private DcMotor[] rightMotors = null;
    private DcMotor[] driveMotors = null;
    private DcMotor[] shooterMotors = null;



    protected void initHardware() {
        initDriveMotors();
      //  initShooterMotors();
      //  initServo();
        initIMU();

    }

    protected DcMotor getFrontLeftMotor() {
        return frontLeftMotor;
    }

    protected DcMotor getFrontRightMotor() {
        return frontRightMotor;
    }

    protected DcMotor getRearLeftMotor() {
        return rearLeftMotor;
    }

    protected DcMotor getRearRightMotor() {
        return rearRightMotor;
    }

    protected DcMotor getLeftShooterMotor() {
        return leftShooterMotor;
    }

    protected DcMotor getRightShooterMotor() {
        return rightShooterMotor;
    }

    protected DcMotor[] getFrontMotors(){
        return frontMotors == null ? new DcMotor[0] : frontMotors.clone();
    }

    protected DcMotor[] getRearMotors(){
        return frontMotors  == null ? new DcMotor[0] : frontMotors.clone();
    }

    protected DcMotor[] getLeftMotors(){
        return leftMotors  == null ? new DcMotor[0] : leftMotors.clone();
    }

    protected DcMotor[] getRightMotors(){
        return rightMotors  == null ? new DcMotor[0] : rightMotors.clone();
    }
    protected DcMotor[] getDriveMotors(){
        return driveMotors  == null ? new DcMotor[0] : driveMotors.clone();
    }

    protected DcMotor[] getShooterMotors(){
        return shooterMotors  == null ? new DcMotor[0] : shooterMotors.clone();
    }


//    protected void initOnlyFrontMotors() {
//
//        frontLeftMotor = hardwareMap.dcMotor.get("dmfl");
//        frontRightMotor = hardwareMap.dcMotor.get("dmfr");
//
//
//
//        frontMotors = new DcMotor[]{frontLeftMotor, frontRightMotor};
//
//        leftMotors = new DcMotor[]{frontLeftMotor};
//        rightMotors = new DcMotor[]{frontRightMotor};
//        driveMotors = new DcMotor[]{frontRightMotor, frontLeftMotor};
//
//
//        setMotorDirection(DcMotor.Direction.REVERSE, rightMotors);
//        setMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, driveMotors);
//        setMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER, driveMotors);
//    }

    protected void initDriveMotors() {

        frontLeftMotor = hardwareMap.dcMotor.get("dmfl");
        frontRightMotor = hardwareMap.dcMotor.get("dmfr");
        rearLeftMotor = hardwareMap.dcMotor.get("dmrl");
        rearRightMotor = hardwareMap.dcMotor.get("dmrr");


        frontMotors = new DcMotor[]{frontLeftMotor, frontRightMotor};
        rearMotors = new DcMotor[]{rearLeftMotor, rearRightMotor};
        leftMotors = new DcMotor[]{frontLeftMotor, rearLeftMotor};
        rightMotors = new DcMotor[]{frontRightMotor, rearRightMotor};
        driveMotors = new DcMotor[]{frontRightMotor, rearRightMotor, frontLeftMotor, rearLeftMotor};


        setMotorDirection(DcMotor.Direction.REVERSE, rightMotors);
        setMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, driveMotors);
        setMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER, driveMotors);
    }

    protected void initShooterMotors() {
        leftShooterMotor = hardwareMap.dcMotor.get("");
        rightShooterMotor = hardwareMap.dcMotor.get("");
        shooterMotors = new DcMotor[]{leftShooterMotor, rightShooterMotor};
    }

    protected void initIMU() {
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

    protected void initServo() {
        servo = hardwareMap.servo.get("slider");
    }

    /**
     * @author Cory
     * @param inches
     * @return the ticks to target to moe this many inches
     */
    public static int ticsForInches(double inches){
        return (int)((inches*TICS_PER_REV)/(Math.PI*WHEEL_DIAMETER_IN));
    }


    public static void setMotorDirection(DcMotor.Direction direction, DcMotor... motors) {
        if (motors == null || motors.length == 0 || direction == null) {
            return;
        }
        for (DcMotor motor : motors) {
            motor.setDirection(direction);
        }
    }

    public static void setMotorRunMode(DcMotor.RunMode runMode, DcMotor... motors) {
        if (motors == null || motors.length == 0 || runMode == null) {
            return;
        }
        for (DcMotor motor : motors) {
            motor.setMode(runMode);
        }
    }

    public static void setMotorTarget(int ticks,@NonNull DcMotor... motors) {
        if (motors == null || motors.length == 0) {
            return;
        }
        for (DcMotor motor : motors) {
            motor.setTargetPosition(ticks);
        }
    }

    public static void setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior zpb, @NonNull DcMotor... motors) {
        if (motors == null || motors.length == 0 || zpb == null) {
            return;
        }
        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(zpb);
        }
    }

    public static void setMotorPower(double power, DcMotor... motors) {
        if (motors == null || motors.length == 0) {
            return;
        }
        double p = Range.clip(power, -1, 1);
        for (DcMotor motor : motors) {
            motor.setPower(p);
        }
    }

    /**
     * Set ZeroPowerBehavior to FLOAT and set power to 0
     * @param motors
     */
    public static void floatMotors(DcMotor ... motors){
        if (motors == null || motors.length == 0) {
            return;
        }
        setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        setMotorPower(0, motors);
    }

    public static int getAveragePosition(DcMotor... motors) {
        int sum = 0;
        for (DcMotor motor : motors) {
            sum += motor.getCurrentPosition();
        }
        return (int) (1.0 * sum / motors.length);
    }

    public static int getMaxPosition(DcMotor... motors) {
        int max = Integer.MIN_VALUE;
        for (DcMotor motor : motors) {
            if (motor.getCurrentPosition() > max) {
                max = motor.getCurrentPosition();
            }
        }
        return max;
    }

    public static boolean someMotorsAreBusy(DcMotor... motors) {
        for (DcMotor motor : motors) {
            if (motor.isBusy()) {
                return true;
            }
        }
        return false;
    }

    /**
     * Set the correct motor mode before calling this
     * @param left
     * @param right
     */
    public void tankDrive(double left, double right){
        setMotorPower(Range.clip(left, -1,1), leftMotors);
        setMotorPower(Range.clip(right, -1,1), rightMotors);
    }

    /**
     * Set the correct motor mode before calling this
     * @param left
     * @param right
     */
    protected void squaredTankDrive(double left, double right){
        setMotorPower(Range.clip(Math.signum(left)*left*left, -1,1), leftMotors);
        setMotorPower(Range.clip(Math.signum(right)*right*right, -1,1), rightMotors);
    }


    /**
     * this blocks until target is reached, assumes encoded motors
     *
     * @param inches the number of inches to travel
     */
    void runStraightForInches(double inches) {
        runStraightToTarget(ticsForInches(inches));
    }

    protected int[] getMotorPositions( DcMotor ... motors){
        if (motors == null || motors.length == 0 ){ return new int[0];}
        int[] ret = new int[motors.length];
        for ( int i = 0 ; i < motors.length; i++ ){
            ret[i] = motors[i].getCurrentPosition();
        }
        return ret;
    }


    /**
     * this blocks until target is reached, assumes encoded motors
     *
     * Motors are set to DcMotor.RunMode.RUN_USING_ENCODER at the end
     * @param ticks
     */
    protected void runStraightToTarget(int ticks) {
        setMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, driveMotors);
        setMotorRunMode(DcMotor.RunMode.RUN_TO_POSITION, driveMotors);
        setMotorTarget(ticks, driveMotors);

        int pos = getAveragePosition(driveMotors);
        while (super.opModeIsActive() && someMotorsAreBusy(driveMotors)) {
            pos = getAveragePosition(driveMotors);
            telemetry.addData("target", ticks);
            telemetry.addData("ave.pos", Arrays.toString(getMotorPositions(driveMotors)));
            telemetry.addData("power", driveMotors[0].getPower());
            // for ramping setMotorPower(.1 + Math.abs(1.0 * pos / ticks), driveMotors);
            //otherwise just go slow
            setMotorPower(.25, driveMotors);
            Thread.yield();
            if (Math.abs(ticks - pos) < 10) {
                break;
            }
        }
        setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER, driveMotors);
    }


    /**
     *
     * @return current heading normalized to +-180 degrees
     */
    private double getCurrentHeading(){
        return  AngleUnit.normalizeDegrees(imu.getAngularOrientation()
                .toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZXY).firstAngle);
    }

    /**
     *
     * @param angle input angle
     * @return Angle in range +-180 degrees
     */
    public static double normalize(double angle) {
        return AngleUnit.normalizeDegrees(angle);
    }

    /**
     * This works for angles where the difference less than +-180. The smaller of
     * the two angles is returned
     *
     * @param targetHeading  The heading at the start of the turn
     * @param currentHeading The current heading
     * @return The sign and magnitude of the angle needed to turn
     */
    public static double calculateDelta(double targetHeading, double currentHeading) {
        double ih = normalize(targetHeading);
        double ch = normalize(currentHeading);

        return  normalize(ih - ch);
    }

    /**
     * <h2>Assumes imu</h2>
     * The coordinate system seems to be increasing counter clockwise, like trig.
     *<p>
     * For counter clockwise turn request a positive number,
     * for clockwise request a negative number.
     * </p>
     *<p>
     * @param degreesRequest the degrees to turn
     *
     */
    protected void turnIMU(double degreesRequest) {
        telemetry.addData("starting turn", degreesRequest);
        double initialHeading = getCurrentHeading();
        double targetHeading = normalize(initialHeading + degreesRequest);

        while (super.opModeIsActive()) {

            double delta = calculateDelta(targetHeading, getCurrentHeading());
            if (Math.abs(delta) > 1) {
                break;
            }
            double power = Math.abs(delta/degreesRequest);
            power = Range.clip(power, MAX_TURNING_POWER, MIN_TURNING_POWER);
            setMotorPower(power * Math.signum(delta), leftMotors);
            setMotorPower(-power * Math.signum(delta), rightMotors);
            telemetry.addData("initial", initialHeading);
            telemetry.addData("target", targetHeading);
            telemetry.addData("delta", delta);
            telemetry.addData("power", power);
        }

        telemetry.addData("ended turn", degreesRequest);
    }


}
