package org.firstinspires.ftc.teamExperiment;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


import java.util.Arrays;


/**
 * Created by clint on 12/4/16.
 */

public class DrivetrainFunctions {

    static final int TICS_PER_REV = 1120;
    static final double WHEEL_DIAMETER = 4;

    static final double MAX_TURNING_POWER = .5;
    static final double MIN_TURNING_POWER = .1;


    /**
     * this blocks until target is reached, assumes encoded motors
     * <p>
     * Motors are set to DcMotor.RunMode.RUN_USING_ENCODER at the end
     *
     * @param ticks
     */
    public static void runStraightToTarget(int ticks, LinearOpMode opmode, Drivetrain dt) {
        setMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, dt.getDriveMotors());
        setMotorRunMode(DcMotor.RunMode.RUN_TO_POSITION, dt.getDriveMotors());
        setMotorTarget(ticks, dt.getDriveMotors());

        int pos = getAveragePosition(dt.getDriveMotors());
        while (opmode.opModeIsActive() && someMotorsAreBusy(dt.getDriveMotors())) {
            pos = getAveragePosition(dt.getDriveMotors());
            dt.getTelemetry().addData("target", ticks);
            dt.getTelemetry().addData("ave.pos", Arrays.toString(getMotorPositions(dt.getDriveMotors())));
            dt.getTelemetry().addData("power", dt.getDriveMotors()[0].getPower());
            dt.getTelemetry().update();
            // for ramping setMotorPower(.1 + Math.abs(1.0 * pos / ticks), driveMotors);
            //otherwise just go slow
            setMotorPower(.25, dt.getDriveMotors());
            Thread.yield();
            if (Math.abs(ticks - pos) < 10) {
                break;
            }
        }
        setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER, dt.getDriveMotors());
    }


    /**
     * this blocks until target is reached, assumes encoded motors
     *
     * @param inches the number of inches to travel
     */
    public static void runStraightForInches(double inches, LinearOpMode opmode, Drivetrain dt) {
        runStraightToTarget(ticsForInches(inches), opmode, dt);
    }

    public static int[] getMotorPositions(DcMotor... motors) {
        if (motors == null || motors.length == 0) {
            return new int[0];
        }
        int[] ret = new int[motors.length];
        for (int i = 0; i < motors.length; i++) {
            ret[i] = motors[i].getCurrentPosition();
        }
        return ret;
    }

    /**
     * Set the correct motor mode before calling this
     *
     * @param left
     * @param right
     */
    public static void tankDrive(double left, double right, Drivetrain dt) {
        setMotorPower(Range.clip(left, -1, 1), dt.getLeftMotors());
        setMotorPower(Range.clip(right, -1, 1), dt.getRightMotors());
    }

    /**
     * Set the correct motor mode before calling this
     *
     * @param left
     * @param right
     * @param dt
     */
    public static void squaredTankDrive(double left, double right, Drivetrain dt) {
        setMotorPower(Range.clip(Math.signum(left) * left * left, -1, 1), dt.getLeftMotors());
        setMotorPower(Range.clip(Math.signum(right) * right * right, -1, 1), dt.getRightMotors());
    }

    /**
     * @return current heading normalized to +-180 degrees
     */
    public static double getCurrentHeading(BNO055IMU imu) {
        return AngleUnit.normalizeDegrees(imu.getAngularOrientation()
                .toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZXY).firstAngle);
    }

    /**
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

        return normalize(ih - ch);
    }

    /**
     * @param inches
     * @return the ticks to target to moe this many inches
     * @author Corey
     */
    public static int ticsForInches(double inches) {
        return (int) ((inches * TICS_PER_REV) / (Math.PI * WHEEL_DIAMETER));
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

    public static void setMotorTarget(int ticks, DcMotor... motors) {
        if (motors == null || motors.length == 0) {
            return;
        }
        for (DcMotor motor : motors) {
            motor.setTargetPosition(ticks);
        }
    }

    public static void setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior zpb, DcMotor... motors) {
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
     *
     * @param motors
     */
    public static void floatMotors(DcMotor... motors) {
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
     * <h2>Assumes imu</h2>
     * The coordinate system seems to be increasing counter clockwise, like trig.
     * <p>
     * For counter clockwise turn request a positive number,
     * for clockwise request a negative number.
     * </p>
     * <p>
     *
     * @param degreesRequest the degrees to turn
     *
     */
    public static void turnIMU(double degreesRequest, LinearOpMode opmode, Hardware robot) {
        double initialHeading = getCurrentHeading(robot.getIMU());
        double targetHeading = normalize(initialHeading + degreesRequest);
        double lastSign = Math.signum(1);
        int switches = 0;
        final long startTime = System.currentTimeMillis();
        final long waitLimit = 5000;

        while (opmode.opModeIsActive()) {
            double delta = calculateDelta(targetHeading, getCurrentHeading(robot.getIMU()));
            if (Math.abs(delta) < .25) {
                setMotorPower(0, robot.getLeftMotors());
                setMotorPower(0, robot.getRightMotors());
                break;
            }
            double power = Math.abs(delta / degreesRequest);
            power = power*power;
            power = Range.clip(power, MIN_TURNING_POWER,MAX_TURNING_POWER );
            setMotorPower(-power * Math.signum(delta), robot.getLeftMotors());
            setMotorPower(power * Math.signum(delta), robot.getRightMotors());
        }
    }
}
