package org.firstinspires.ftc.team9374;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by darwin on 11/29/16.
 */

//  must be set up such that center of the robot is aligned with the square directly to the left.
@Autonomous(name = "Beta_light sensor")

public class BETA_Light_sensor extends LinearOpMode {

    Hardware9374 robot = new Hardware9374();
    public void runOpMode() throws InterruptedException{
        robot.init(hardwareMap);

        robot.moveToPosition(61,.7);

        robot.Turn(90,1,false);

        robot.moveToPosition(67,.7);

    }
}