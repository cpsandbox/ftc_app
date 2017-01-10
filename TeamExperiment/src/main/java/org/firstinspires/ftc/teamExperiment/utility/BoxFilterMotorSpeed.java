package org.firstinspires.ftc.teamExperiment.utility;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Arrays;

/**
 * Must always be called from the same thread
 * Created by clint on 12/9/16.
 */

/**
 * This does not properly capture counts that exceed the filter length
 */
public class BoxFilterMotorSpeed {


    private DcMotor[] motors = null;
    // last tick reading
    private int[][] motorReadings;
    // system.nanos of reading
    private long[][] timings;
    // ticks per second
    private double[] speeds = null;

    private int[] indexes = null;


    public BoxFilterMotorSpeed(int boxes, DcMotor... ms) {
        if (ms == null || ms.length == 0) {
            throw new IllegalArgumentException(" motors < 1");
        }
        this.motors = ms.clone();
        motorReadings = new int[motors.length][boxes];
        this.timings = new long[motors.length][boxes];
        speeds = new double[motors.length];
        indexes = new int[motors.length];
    }

    public void read() {

        for (int i = 0; i < motors.length; i++) {
            int read = motors[i].getCurrentPosition();
            long ct = System.nanoTime();

            int ix = indexes[i];

            int ix_1 = (ix + timings[i].length -1)%timings[i].length;
            int ix1 = (ix +1)%timings[i].length;

            if (motorReadings[i][ix_1] != read) {

                motorReadings[i][ix] = read;


                timings[i][ix] = ct;



                speeds[i] = 1e9 * (motorReadings[i][ix1]-motorReadings[i][ix]) / (timings[i][ix1]-timings[i][ix]);

                indexes[i] = ix1;

            }
        }
    }

    /**
     * In ticks per second
     *
     * @return
     */
    public double[] getSpeeds() {
        return speeds;
    }
}