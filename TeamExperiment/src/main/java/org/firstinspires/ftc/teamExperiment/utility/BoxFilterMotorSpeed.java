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
    private long[][] motorReadings;
    private long[] timings;
    private int index = 0;


    public BoxFilterMotorSpeed(int boxSize, DcMotor... ms) {
        if (boxSize < 1 || ms == null || ms.length == 0) {
            throw new IllegalArgumentException("boxSize < 1 or motors < 1");
        }
        this.motors = ms.clone();
        motorReadings = new long[motors.length][boxSize];
        this.timings = new long[boxSize];

    }

    public void read() {
        timings[index] = System.nanoTime();
        for (int i = 0; i < motors.length; i++) {
            motorReadings[i][index] = motors[i].getCurrentPosition();
        }
        index = (index + 1) % timings.length;
    }

    /**
     * ticks/second
     *
     * @return
     */
//    public double[] getSpeeds() {
//
//
//        double[] mt = new double[motors.length];
//        for ( int i = 0; i < timings.length-1; i++){
//            int ix0 = (index + i)%timings.length;
//            int ix1 = (index + i + 1)%timings.length;
//            double dt = timings[ix1] - timings[ix0];
//
//            for ( int j = 0; j < motorReadings.length; j++ ){
//                mt[j]+= (motorReadings[j][ix1] - motorReadings[j][ix0])/dt;
//            }
//        }


//
//        double[] ret = new double[motors.length];
//        for (int i = 0; i < ret.length; i++) {
//            ret[i] = mt[i] / (timings.length - 1) * 1e-9 /* seconds/nano */;
//        }
//        return ret;
//    }


    /**
     * Ticks per second
     * @return
     */
    public double[] getSpeeds() {
        double[] mt = new double[motors.length];
        int ix0 = (index - 1 + timings.length) % timings.length;
        int ix1 = (index);
        double dt = Math.max(1, timings[ix1] - timings[ix0])* 1e-9;
        for (int j = 0; j < motorReadings.length; j++) {
            mt[j] += (motorReadings[j][ix1] - motorReadings[j][ix0]) / dt;
        }
        return mt;
    }

    /**
     * in ms
     *
     * @return
     */
    public long getDT() {
        int ix0 = (index - 1 + timings.length) % timings.length;
        int ix1 = (index);
        return (long) (timings[ix1] - timings[ix0] / 1e6 /*nanoseconds per ms*/);
    }

}
