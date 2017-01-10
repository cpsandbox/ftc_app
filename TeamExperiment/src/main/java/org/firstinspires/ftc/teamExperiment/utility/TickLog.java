package org.firstinspires.ftc.teamExperiment.utility;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.AppUtil;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Arrays;

/**
 * Created by clint on 12/11/16.
 */

public class TickLog {

    private DcMotor[] motors = null;
    private int[][] motorReadings;
    private long[][] timings;
    private int[] index = null;


    public TickLog( DcMotor... ms) {
        if ( ms == null || ms.length == 0) {
            throw new IllegalArgumentException(" motors < 1");
        }
        this.motors = ms.clone();
        motorReadings = new int[motors.length][100_000];
        this.timings = new long[motors.length][100_000];
        index = new int[motors.length];
        for( int i = 0;i < index.length; i++){
            index[i]=1;
        }

    }

    public void read() {
        // resize if needed

        for (int i = 0; i < index.length; i++) {
            int read = motors[i].getCurrentPosition();
            long ct = System.nanoTime();
            if ( motorReadings[i][index[i]-1] != read ){

                if ( index[i] == timings[i].length ){
                    int newLength = Math.min(timings[i].length*2, timings[i].length + 100_000);
                    timings[i] = Arrays.copyOf(timings[i],newLength);
                    motorReadings[i] = Arrays.copyOf(motorReadings[i], newLength);
                }

                motorReadings[i][index[i]] = read;
                timings[i][index[i]] = ct;
                index[i]++;
            }
        }
    }

    public boolean writeData(String f){
        File file = AppUtil.getInstance().getSettingsFile(f);
        PrintWriter pw = null;
        int maxIndex = 0;
        try {
            pw =new PrintWriter(new FileWriter(file));
            for ( int i =0; i < index.length; i++) {
                if ( index[i] > maxIndex){maxIndex = index[i];}
                if ( i > 0 ){pw.print("\t");}
                pw.print("Time (ns)\tMotor " +(i+1)+" Clicks" );

            }
            pw.println();


            for ( int i =0; i < maxIndex; i++) {
                for ( int j =0; j < index.length;j++) {
                    if (j > 0) {
                        pw.print("\t");
                    }

                    if (i < index[i]) {
                        pw.print(timings[j][i] + "\t" + motorReadings[j][i]);
                    } else {
                        pw.print("\t");
                    }

                }
                    pw.println();
            }

            return true;
        } catch(IOException ioe){
            System.out.println("failed to write TickLog to " + f);
        }
        finally{
            if (pw != null) {
                pw.close();
            }
        }
        return false;
    }
}
