package org.firstinspires.ftc.teamExperiment.utility;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Arrays;

/**
 * Created by clint on 12/12/16.
 */

public class MotorSpeed {


    private DcMotor[] motors = null;
    // last tick reading
    private int[] motorReadings;
    // system.nanos of reading
    private long[] timings;
    // ticks per second
    private double[] speeds = null;


    public MotorSpeed( DcMotor... ms) {
        if ( ms == null || ms.length == 0) {
            throw new IllegalArgumentException(" motors < 1");
        }
        this.motors = ms.clone();
        motorReadings = new int[motors.length];
        this.timings = new long[motors.length];
        speeds = new double[motors.length];

    }

    public void read() {

        for (int i = 0; i < motors.length; i++) {
            int read = motors[i].getCurrentPosition();
            long ct = System.nanoTime();
            if ( motorReadings[i] != read ){
                int dc = read - motorReadings[i];
                motorReadings[i] = read;
                double dt = ct - timings[i];
                timings[i] = ct;
                speeds[i] = dc/dt * 1e9;

            }
        }
    }

    /**
     * In ticks per second
     * @return
     */
    public double[] getSpeeds(){
        return speeds;
    }
}
